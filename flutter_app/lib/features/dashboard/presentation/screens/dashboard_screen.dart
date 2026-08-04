import 'dart:async';
import 'dart:convert';
import 'package:flutter/material.dart';
import 'package:clay_containers/clay_containers.dart';
import 'package:flutter/services.dart';
import 'package:font_awesome_flutter/font_awesome_flutter.dart';
import 'package:flutter/cupertino.dart';
import 'package:shared_preferences/shared_preferences.dart';

import '../../../../core/config/config.dart';
import '../../../../core/services/mqtt_service.dart';
import '../../../auth/presentation/screens/login_screen.dart';
import '../widgets/settings_modal.dart';

class DashboardPage extends StatefulWidget {
  final String brokerAddress;
  final int port;
  final String username;
  final String password;
  final String deviceId;

  const DashboardPage({
    super.key,
    required this.brokerAddress,
    required this.port,
    required this.username,
    required this.password,
    required this.deviceId,
  });

  @override
  State<DashboardPage> createState() => _DashboardPageState();
}

class _DashboardPageState extends State<DashboardPage>
    with WidgetsBindingObserver {
  DateTime? _lastBackPressTime;
  final int _exitThreshold = 2000;

  late final MQTTService _mqttService;

  double _currentBrightness = 0.0;
  bool _isPowerOn = false;
  MQTTConnectionStatus _connectionStatus = MQTTConnectionStatus.disconnected;

  String _powerConsumption = '0 kWh';
  String _voltage = '0 V';
  String _current = '0 A';
  String _power = '0 W';
  String _frequency = '0 Hz';
  String _powerFactor = '0.0';

  @override
  void initState() {
    super.initState();
    WidgetsBinding.instance.addObserver(this);

    _mqttService = MQTTService(
      onMessageReceived: _handleIncomingMessage,
      onConnectionStatusChanged: (bool connected) {
        if (!mounted) return;
        setState(() {
          _connectionStatus = connected
              ? MQTTConnectionStatus.connected
              : MQTTConnectionStatus.disconnected;
        });
        if (!connected) _attemptReconnect();
      },
      onError: _handleError,
    );

    _mqttService.configure(
      broker: widget.brokerAddress,
      port: widget.port,
      username: widget.username,
      password: widget.password,
      deviceId: widget.deviceId,
    );

    _initMQTT();
  }

  void _showSettingsModal() {
    showDialog(
      context: context,
      barrierDismissible: true,
      builder: (context) => SettingsModal(
        theme: ThemeNotifier(),
        deviceId: widget.deviceId,
        brokerAddress: widget.brokerAddress,
        port: widget.port,
        username: widget.username,
        connectionStatus: _connectionStatus,
        onLogout: _logout,
      ),
    );
  }

  @override
  void didChangeAppLifecycleState(AppLifecycleState state) {
    if (state == AppLifecycleState.paused) {
      _mqttService.disconnect();
    } else if (state == AppLifecycleState.resumed) {
      Future.delayed(const Duration(milliseconds: 500), () => _initMQTT());
    }
  }

  Future<void> _initMQTT() async {
    try {
      if (mounted) {
        setState(() => _connectionStatus = MQTTConnectionStatus.connecting);
      }
      await _mqttService.connect();
      await _mqttService.subscribe('devices/${widget.deviceId}/telemetry');
      await _mqttService.subscribe('devices/${widget.deviceId}/status');
    } catch (e) {
      if (mounted) {
        setState(() => _connectionStatus = MQTTConnectionStatus.error);
      }
    }
  }

  void _attemptReconnect() async {
    if (WidgetsBinding.instance.lifecycleState == AppLifecycleState.paused) {
      return;
    }
    await Future.delayed(const Duration(seconds: 3));
    if (!_mqttService.isConnected) _initMQTT();
  }

  Future<void> _logout() async {
    final prefs = await SharedPreferences.getInstance();
    await prefs.clear();

    if (!mounted) return;
    _mqttService.disconnect();
    Navigator.of(context).pushAndRemoveUntil(
      MaterialPageRoute(builder: (_) => const LoginPage()),
      (route) => false,
    );
  }

  void _handleIncomingMessage(String payload) {
    if (!mounted) return;
    try {
      final jsonData = json.decode(payload);
      setState(() {
        if (jsonData['voltage'] != null) _voltage = '${jsonData['voltage']} V';
        if (jsonData['current'] != null) _current = '${jsonData['current']} mA';
        if (jsonData['power'] != null) {
          if (jsonData['power'] is bool) {
            _isPowerOn = jsonData['power'] as bool;
          } else {
            _power = '${jsonData['power']} W';
          }
        }
        if (jsonData['brightness'] != null && jsonData['brightness'] is num) {
          _currentBrightness = (jsonData['brightness'] as num).toDouble();
        }
        if (jsonData['frequency'] != null) {
          _frequency = '${jsonData['frequency']} Hz';
        }
        if (jsonData['powerConsumption'] != null) {
          _powerConsumption = '${jsonData['powerConsumption']} kWh';
        }
        if (jsonData['powerFactor'] != null) {
          _powerFactor = jsonData['powerFactor'].toString();
        }
      });
    } catch (_) {}
  }

  void _handleError(String error) {
    if (!mounted) return;
    setState(() => _connectionStatus = MQTTConnectionStatus.error);
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Text(error),
        duration: const Duration(seconds: 5),
        action: SnackBarAction(label: 'RETRY', onPressed: _initMQTT),
      ),
    );
  }

  void _publishState() {
    if (_mqttService.isConnected) {
      final payload = json.encode({
        "brightness": _currentBrightness.toInt(),
        "power": _isPowerOn,
      });
      _mqttService.publish('devices/${widget.deviceId}/command', payload);
    }
  }

  void _togglePower() {
    HapticFeedback.lightImpact();
    setState(() {
      _isPowerOn = !_isPowerOn;
      if (_isPowerOn && _currentBrightness < 10) _currentBrightness = 100;
      if (!_isPowerOn) _currentBrightness = 0;
    });
    _publishState();
  }

  void _onSliderChanged(double value) {
    setState(() {
      _currentBrightness = value;
      _isPowerOn = value > 0;
    });
  }

  void _onSliderEnd(double value) {
    _publishState();
  }

  @override
  void dispose() {
    WidgetsBinding.instance.removeObserver(this);
    _mqttService.disconnect();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return ListenableBuilder(
      listenable: ThemeNotifier(),
      builder: (context, _) {
        final theme = ThemeNotifier();
        return PopScope(
          canPop: false,
          onPopInvokedWithResult: (didPop, result) async {
            if (didPop) return;
            final now = DateTime.now();
            if (_lastBackPressTime == null ||
                now.difference(_lastBackPressTime!) >
                    Duration(milliseconds: _exitThreshold)) {
              _lastBackPressTime = now;
              ScaffoldMessenger.of(context).showSnackBar(
                const SnackBar(
                  content: Text('Press back again to exit'),
                  duration: Duration(seconds: 2),
                ),
              );
            } else {
              SystemNavigator.pop();
            }
          },
          child: AnnotatedRegion<SystemUiOverlayStyle>(
            value: SystemUiOverlayStyle(
              statusBarColor: Colors.transparent,
              statusBarIconBrightness: theme.isDark
                  ? Brightness.light
                  : Brightness.dark,
              systemNavigationBarColor: theme.primaryColor,
              systemNavigationBarIconBrightness: theme.isDark
                  ? Brightness.light
                  : Brightness.dark,
            ),
            child: Scaffold(
              extendBodyBehindAppBar: true,
              backgroundColor: theme.primaryColor,
              body: Container(
                decoration: BoxDecoration(
                  gradient: LinearGradient(
                    begin: Alignment.topCenter,
                    end: Alignment.bottomCenter,
                    colors: [theme.secondaryColor, theme.primaryColor],
                    stops: const [0.0, 0.3],
                  ),
                ),
                child: SafeArea(
                  child: SingleChildScrollView(
                    padding: const EdgeInsets.symmetric(vertical: 16),
                    child: Column(
                      children: [
                        const SizedBox(height: 12),
                        _buildTopBar(theme),
                        const SizedBox(height: 12),
                        _buildPowerConsumptionCard(theme),
                        const SizedBox(height: 16),
                        _buildLightBulb(theme),
                        const SizedBox(height: 16),
                        _buildBrightnessIndicator(theme),
                        const SizedBox(height: 8),
                        _buildBrightnessSlider(theme),
                        const SizedBox(height: 16),
                        _buildSensorGrid(theme),
                        const SizedBox(height: 26),
                      ],
                    ),
                  ),
                ),
              ),
            ),
          ),
        );
      },
    );
  }

  Widget _buildTopBar(ThemeNotifier theme) {
    return Padding(
      padding: const EdgeInsets.all(16.0),
      child: Row(
        mainAxisAlignment: MainAxisAlignment.spaceBetween,
        children: [
          ClayContainer(
            height: 40,
            width: 40,
            borderRadius: 10,
            color: theme.primaryColor,
            depth: theme.depth,
            child: InkWell(
              onTap: _showSettingsModal,
              child: Center(
                child: Icon(
                  Icons.tune_rounded,
                  color: theme.iconColor,
                  size: kMenuIconSize,
                ),
              ),
            ),
          ),
          Row(
            children: [
              Container(
                width: 12,
                height: 12,
                margin: const EdgeInsets.only(right: 8),
                decoration: BoxDecoration(
                  color: _connectionStatus == MQTTConnectionStatus.connected
                      ? Colors.green
                      : Colors.red,
                  shape: BoxShape.circle,
                ),
              ),
              Text(
                _connectionStatus.name,
                style: TextStyle(
                  color: theme.textColor,
                  fontSize: 24,
                  fontWeight: FontWeight.w600,
                ),
              ),
            ],
          ),
          ClayContainer(
            height: 40,
            width: 40,
            borderRadius: 10,
            color: theme.primaryColor,
            depth: theme.depth,
            child: InkWell(
              onTap: _togglePower,
              child: Center(
                child: Icon(
                  CupertinoIcons.power,
                  color: _isPowerOn ? Colors.red : theme.iconColor,
                  size: kPowerIconSize,
                ),
              ),
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildPowerConsumptionCard(ThemeNotifier theme) {
    return Container(
      height: 120,
      padding: const EdgeInsets.symmetric(vertical: 6),
      child: Center(
        child: SizedBox(
          width: MediaQuery.of(context).size.width * 0.8,
          child: ClayContainer(
            borderRadius: 16,
            color: theme.primaryColor,
            depth: theme.depth,
            child: Container(
              decoration: BoxDecoration(
                gradient: const LinearGradient(colors: gradientColors),
                borderRadius: BorderRadius.circular(16),
              ),
              padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 10),
              child: Row(
                children: [
                  const FaIcon(
                    FontAwesomeIcons.plugCircleBolt,
                    color: Colors.white,
                    size: 24,
                  ),
                  const SizedBox(width: 12),
                  Column(
                    mainAxisAlignment: MainAxisAlignment.center,
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      Text(
                        theme.translate('power_consumption').toUpperCase(),
                        style: const TextStyle(
                          color: Colors.white70,
                          fontSize: 12,
                          fontWeight: FontWeight.bold,
                        ),
                      ),
                      const SizedBox(height: 4),
                      AnimatedValue(
                        theme.formatNumber(_powerConsumption),
                        style: const TextStyle(
                          color: Colors.white,
                          fontSize: 22,
                          fontWeight: FontWeight.bold,
                        ),
                      ),
                    ],
                  ),
                ],
              ),
            ),
          ),
        ),
      ),
    );
  }

  Widget _buildLightBulb(ThemeNotifier theme) {
    return ClayContainer(
      height: 180,
      width: 180,
      borderRadius: 200,
      color: theme.primaryColor,
      depth: theme.depth,
      child: Stack(
        alignment: Alignment.center,
        children: [
          Icon(
            _isPowerOn
                ? CupertinoIcons.lightbulb_fill
                : CupertinoIcons.lightbulb,
            size: kLightbulbSize,
            color: _isPowerOn
                ? Colors.orangeAccent
                : theme.subTextColor.withValues(alpha: 0.3),
          ),
          if (_isPowerOn)
            Container(
              height: 200,
              width: 200,
              decoration: BoxDecoration(
                borderRadius: BorderRadius.circular(200),
                boxShadow: [
                  BoxShadow(
                    color: Colors.deepOrange.withValues(
                      alpha: _currentBrightness / 250,
                    ),
                    blurRadius: _currentBrightness / 2,
                    spreadRadius: _currentBrightness / 6,
                  ),
                ],
              ),
            ),
        ],
      ),
    );
  }

  Widget _buildBrightnessIndicator(ThemeNotifier theme) {
    return Padding(
      padding: const EdgeInsets.symmetric(horizontal: 24.0),
      child: Row(
        mainAxisAlignment: MainAxisAlignment.spaceBetween,
        children: [
          Text(
            theme.translate('brightness'),
            style: TextStyle(
              color: theme.subTextColor,
              fontSize: 18,
              fontWeight: FontWeight.w600,
            ),
          ),
          Container(
            padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 6),
            decoration: BoxDecoration(
              color: theme.secondaryColor,
              borderRadius: BorderRadius.circular(20),
            ),
            child: Text(
              '${theme.formatNumber(_currentBrightness.toInt())}%',
              style: TextStyle(
                color: theme.textColor,
                fontSize: 16,
                fontWeight: FontWeight.bold,
              ),
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildBrightnessSlider(ThemeNotifier theme) {
    return Padding(
      padding: const EdgeInsets.symmetric(horizontal: 20.0),
      child: SliderTheme(
        data: SliderTheme.of(context).copyWith(
          activeTrackColor: activeColor2,
          inactiveTrackColor: theme.subTextColor.withValues(alpha: 0.3),
          thumbColor: activeColor1,
          thumbShape: const RoundSliderThumbShape(
            enabledThumbRadius: 12,
            elevation: 6,
          ),
          overlayColor: activeColor2.withValues(alpha: 0.2),
          trackHeight: 6,
        ),
        child: Slider(
          min: 0,
          max: 100,
          divisions: 100,
          value: _currentBrightness,
          onChanged: _onSliderChanged,
          onChangeEnd: _onSliderEnd,
        ),
      ),
    );
  }

  Widget _buildSensorGrid(ThemeNotifier theme) {
    return Padding(
      padding: const EdgeInsets.symmetric(horizontal: 16),
      child: GridView.count(
        physics: const NeverScrollableScrollPhysics(),
        shrinkWrap: true,
        crossAxisCount: 2,
        childAspectRatio: 1.6,
        mainAxisSpacing: 12,
        crossAxisSpacing: 12,
        children: [
          _sensorCard(
            theme,
            CupertinoIcons.bolt_fill,
            theme.translate('voltage'),
            theme.formatNumber(_voltage),
            [
              activeColor1.withValues(alpha: 0.7),
              activeColor2.withValues(alpha: 0.5),
            ],
          ),
          _sensorCard(
            theme,
            CupertinoIcons.bolt_horizontal_fill,
            theme.translate('current'),
            theme.formatNumber(_current),
            [
              activeColor2.withValues(alpha: 0.7),
              activeColor1.withValues(alpha: 0.5),
            ],
          ),
          _sensorCard(
            theme,
            CupertinoIcons.lightbulb_fill,
            theme.translate('power'),
            theme.formatNumber(_power),
            [
              Colors.orange.withValues(alpha: 0.7),
              Colors.red.withValues(alpha: 0.5),
            ],
          ),
          _sensorCard(
            theme,
            CupertinoIcons.graph_square_fill,
            theme.translate('frequency'),
            theme.formatNumber(_frequency),
            [
              Colors.purple.withValues(alpha: 0.7),
              Colors.blue.withValues(alpha: 0.5),
            ],
          ),
          _sensorCard(
            theme,
            Icons.show_chart,
            theme.translate('power_factor'),
            theme.formatNumber(_powerFactor),
            [
              Colors.green.withValues(alpha: 0.7),
              Colors.teal.withValues(alpha: 0.5),
            ],
          ),
          _sensorCard(
            theme,
            Icons.device_hub,
            theme.translate('device_id_label'),
            widget.deviceId.length > 12
                ? '${widget.deviceId.substring(0, 12)}...'
                : widget.deviceId,
            [
              Colors.grey.withValues(alpha: 0.7),
              Colors.blueGrey.withValues(alpha: 0.5),
            ],
          ),
        ],
      ),
    );
  }

  Widget _sensorCard(
    ThemeNotifier theme,
    IconData icon,
    String label,
    String value,
    List<Color> gradient,
  ) {
    return ClayContainer(
      color: theme.primaryColor,
      borderRadius: 12,
      depth: theme.depth,
      child: Container(
        decoration: BoxDecoration(
          gradient: LinearGradient(
            colors: gradient,
            begin: Alignment.topLeft,
            end: Alignment.bottomRight,
          ),
          borderRadius: BorderRadius.circular(12),
        ),
        padding: const EdgeInsets.all(12),
        child: Column(
          mainAxisAlignment: MainAxisAlignment.spaceBetween,
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(
              children: [
                Icon(icon, color: Colors.white70, size: 20),
                const SizedBox(width: 8),
                Text(
                  label,
                  style: const TextStyle(color: Colors.white, fontSize: 14),
                ),
              ],
            ),
            Align(
              alignment: Alignment.bottomRight,
              child: AnimatedValue(
                value,
                style: const TextStyle(
                  color: Colors.white,
                  fontSize: 18,
                  fontWeight: FontWeight.bold,
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }
}

class AnimatedValue extends StatelessWidget {
  final String text;
  final TextStyle? style;
  const AnimatedValue(this.text, {super.key, this.style});
  @override
  Widget build(BuildContext context) {
    return AnimatedSwitcher(
      duration: const Duration(milliseconds: 200),
      child: Text(text, key: ValueKey(text), style: style),
    );
  }
}

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:shared_preferences/shared_preferences.dart';
import '../../../../core/config/config.dart';
import '../../../../core/constants/app_colors.dart';
import '../../../../core/services/mqtt_service.dart';
import '../../../dashboard/presentation/screens/dashboard_screen.dart';
import '../widgets/auth_text_field.dart';

class LoginPage extends StatefulWidget {
  const LoginPage({super.key});

  @override
  State<LoginPage> createState() => _LoginPageState();
}

class _LoginPageState extends State<LoginPage> {
  final _formKey = GlobalKey<FormState>();

  final _brokerAddressController = TextEditingController();
  final _portController = TextEditingController();
  final _usernameController = TextEditingController();
  final _passwordController = TextEditingController();
  final _deviceIdController = TextEditingController();

  bool _isConnecting = false;
  bool _isLoadingPrefs = true;
  bool _obscurePassword = true;
  String _errorMessage = '';

  @override
  void initState() {
    super.initState();
    _checkAutoLogin();
  }

  Future<void> _checkAutoLogin() async {
    final prefs = await SharedPreferences.getInstance();

    setState(() {
      _brokerAddressController.text = prefs.getString('broker') ?? '';
      _portController.text = prefs.getString('port') ?? '8883';
      _usernameController.text = prefs.getString('username') ?? '';
      _passwordController.text = prefs.getString('password') ?? '';
      _deviceIdController.text = prefs.getString('deviceId') ?? '';
      _isLoadingPrefs = false;
    });

    bool wasLoggedIn = prefs.getBool('isLoggedIn') ?? false;

    if (wasLoggedIn && _deviceIdController.text.isNotEmpty) {
      if (mounted) {
        Navigator.of(context).pushReplacement(
          MaterialPageRoute(
            builder: (context) => DashboardPage(
              brokerAddress: _brokerAddressController.text,
              port: int.tryParse(_portController.text) ?? 8883,
              username: _usernameController.text,
              password: _passwordController.text,
              deviceId: _deviceIdController.text,
            ),
          ),
        );
      }
    }
  }

  Future<void> _saveCredentials() async {
    final prefs = await SharedPreferences.getInstance();
    await prefs.setString('broker', _brokerAddressController.text.trim());
    await prefs.setString('port', _portController.text.trim());
    await prefs.setString('username', _usernameController.text.trim());
    await prefs.setString('password', _passwordController.text.trim());
    await prefs.setString('deviceId', _deviceIdController.text.trim());
    await prefs.setBool('isLoggedIn', true);
  }

  @override
  void dispose() {
    _brokerAddressController.dispose();
    _portController.dispose();
    _usernameController.dispose();
    _passwordController.dispose();
    _deviceIdController.dispose();
    super.dispose();
  }

  Future<void> _connectToBroker() async {
    if (!_formKey.currentState!.validate()) return;

    TextInput.finishAutofillContext();
    FocusScope.of(context).unfocus();

    setState(() {
      _isConnecting = true;
      _errorMessage = '';
    });

    final testService = MQTTService();
    final int port = int.tryParse(_portController.text.trim()) ?? 8883;

    testService.configure(
      broker: _brokerAddressController.text.trim(),
      port: port,
      username: _usernameController.text.trim(),
      password: _passwordController.text.trim(),
      deviceId: _deviceIdController.text.trim(),
    );

    try {
      await testService.connect(maxRetries: 1);

      if (testService.isConnected) {
        await testService.disconnect();
        await _saveCredentials();

        if (mounted) {
          Navigator.of(context).pushReplacement(
            MaterialPageRoute(
              builder: (context) => DashboardPage(
                brokerAddress: _brokerAddressController.text.trim(),
                port: port,
                username: _usernameController.text.trim(),
                password: _passwordController.text.trim(),
                deviceId: _deviceIdController.text.trim(),
              ),
            ),
          );
        }
      }
    } catch (e) {
      if (mounted) {
        setState(() => _errorMessage = 'Connection failed.\n${e.toString()}');
      }
    } finally {
      if (mounted) setState(() => _isConnecting = false);
    }
  }

  void _loginAsGuest() {
    TextInput.finishAutofillContext();
    FocusScope.of(context).unfocus();

    Navigator.of(context).pushReplacement(
      MaterialPageRoute(
        builder: (context) => const DashboardPage(
          brokerAddress: 'demo.emqx.io',
          port: 8883,
          username: 'guest',
          password: '',
          deviceId: 'dimmo',
          isGuestMode: true,
        ),
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    return ListenableBuilder(
      listenable: ThemeNotifier(),
      builder: (context, _) {
        final theme = ThemeNotifier();
        final size = MediaQuery.of(context).size;

        if (_isLoadingPrefs) {
          return Scaffold(
            backgroundColor: theme.primaryColor,
            body: const Center(
              child: CircularProgressIndicator(color: activeColor1),
            ),
          );
        }

        return Scaffold(
          backgroundColor: theme.primaryColor,
          body: GestureDetector(
            behavior: HitTestBehavior.translucent,
            onTap: () {
              FocusScope.of(context).unfocus();
            },
            child: SafeArea(
              child: SingleChildScrollView(
                child: Container(
                  width: size.width,
                  padding: const EdgeInsets.symmetric(horizontal: 24),
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.center,
                    children: [
                      const SizedBox(height: 16),
                      Directionality(
                        textDirection: TextDirection.ltr,
                        child: Row(
                          mainAxisAlignment: MainAxisAlignment.spaceBetween,
                          children: [
                            // Language translation icon fixed on left
                            IconButton(
                              onPressed: theme.toggleLanguage,
                              icon: Icon(
                                Icons.translate,
                                color: theme.textColor,
                                size: 24,
                              ),
                            ),
                            // Theme change icon fixed on right
                            IconButton(
                              onPressed: theme.toggleTheme,
                              icon: Icon(
                                theme.isDark
                                    ? Icons.light_mode_outlined
                                    : Icons.dark_mode_outlined,
                                color: theme.textColor,
                                size: 24,
                              ),
                            ),
                          ],
                        ),
                      ),
                      const SizedBox(height: 20),
                      Text(
                        theme.translate('login_title'),
                        style: TextStyle(
                          fontSize: 32,
                          fontWeight: FontWeight.bold,
                          color: theme.textColor,
                        ),
                      ),
                      const SizedBox(height: 10),
                      Padding(
                        padding: const EdgeInsets.symmetric(horizontal: 40),
                        child: Text(
                          theme.translate('login_subtitle'),
                          textAlign: TextAlign.center,
                          style: TextStyle(
                            fontSize: 16,
                            color: theme.subTextColor,
                            fontWeight: FontWeight.w500,
                          ),
                        ),
                      ),
                      const SizedBox(height: 40),
                      Form(
                        key: _formKey,
                        child: Column(
                          children: [
                            AuthTextField(
                              theme: theme,
                              controller: _brokerAddressController,
                              label: theme.translate('broker_address'),
                              icon: Icons.cloud_outlined,
                              isRequired: true,
                            ),
                            const SizedBox(height: 16),
                            AuthTextField(
                              theme: theme,
                              controller: _portController,
                              label: theme.translate('port_label'),
                              icon: Icons.numbers_outlined,
                              isNumber: true,
                              isRequired: true,
                            ),
                            const SizedBox(height: 16),
                            AuthTextField(
                              theme: theme,
                              controller: _usernameController,
                              label: theme.translate('username_optional'),
                              icon: Icons.person_outline,
                              autofillHints: const [AutofillHints.username],
                            ),
                            const SizedBox(height: 16),
                            AuthTextField(
                              theme: theme,
                              controller: _passwordController,
                              label: theme.translate('password_optional'),
                              icon: Icons.lock_outline,
                              isObscure: _obscurePassword,
                              autofillHints: const [AutofillHints.password],
                              toggleObscure: () => setState(
                                () => _obscurePassword = !_obscurePassword,
                              ),
                            ),
                            const SizedBox(height: 16),
                            AuthTextField(
                              theme: theme,
                              controller: _deviceIdController,
                              label: theme.translate('device_id_label'),
                              icon: Icons.device_hub_outlined,
                              isRequired: true,
                            ),
                            const SizedBox(height: 30),
                            if (_errorMessage.isNotEmpty)
                              Padding(
                                padding: const EdgeInsets.only(bottom: 24),
                                child: Container(
                                  padding: const EdgeInsets.all(12),
                                  decoration: BoxDecoration(
                                    color: AppColors.errorRed.withValues(
                                      alpha: 0.1,
                                    ),
                                    borderRadius: BorderRadius.circular(12),
                                  ),
                                  child: Text(
                                    _errorMessage,
                                    style: const TextStyle(
                                      color: AppColors.errorRed,
                                      fontSize: 13,
                                    ),
                                    textAlign: TextAlign.center,
                                  ),
                                ),
                              ),
                            SizedBox(
                              width: double.infinity,
                              height: 56,
                              child: ElevatedButton(
                                onPressed: _isConnecting
                                    ? null
                                    : _connectToBroker,
                                style: ElevatedButton.styleFrom(
                                  backgroundColor: activeColor1,
                                  foregroundColor: Colors.white,
                                  shape: RoundedRectangleBorder(
                                    borderRadius: BorderRadius.circular(16),
                                  ),
                                  elevation: 6,
                                  shadowColor: activeColor1.withValues(
                                    alpha: 0.4,
                                  ),
                                ),
                                child: _isConnecting
                                    ? const SizedBox(
                                        height: 24,
                                        width: 24,
                                        child: CircularProgressIndicator(
                                          color: Colors.white,
                                          strokeWidth: 2,
                                        ),
                                      )
                                    : Text(
                                        theme.translate('sign_in'),
                                        style: const TextStyle(
                                          fontSize: 18,
                                          fontWeight: FontWeight.bold,
                                          letterSpacing: 1.2,
                                        ),
                                      ),
                              ),
                            ),
                            const SizedBox(height: 14),
                            SizedBox(
                              width: double.infinity,
                              height: 52,
                              child: OutlinedButton.icon(
                                onPressed: _isConnecting ? null : _loginAsGuest,
                                icon: Icon(
                                  Icons.person_outline_rounded,
                                  color: theme.textColor,
                                  size: 20,
                                ),
                                label: Text(
                                  theme.translate('guest_login'),
                                  style: TextStyle(
                                    fontSize: 16,
                                    fontWeight: FontWeight.w600,
                                    color: theme.textColor,
                                    letterSpacing: 0.5,
                                  ),
                                ),
                                style: OutlinedButton.styleFrom(
                                  backgroundColor: theme.isDark
                                      ? const Color(0xFF2C2C2E)
                                      : Colors.white.withValues(alpha: 0.8),
                                  side: BorderSide(
                                    color: theme.subTextColor.withValues(
                                      alpha: 0.25,
                                    ),
                                    width: 1.5,
                                  ),
                                  shape: RoundedRectangleBorder(
                                    borderRadius: BorderRadius.circular(16),
                                  ),
                                  elevation: 2,
                                  shadowColor: Colors.black.withValues(
                                    alpha: theme.isDark ? 0.2 : 0.05,
                                  ),
                                ),
                              ),
                            ),
                            const SizedBox(height: 40),
                          ],
                        ),
                      ),
                    ],
                  ),
                ),
              ),
            ),
          ),
        );
      },
    );
  }
}

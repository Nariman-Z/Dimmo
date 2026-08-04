import 'dart:async';
import 'dart:io';
import 'package:flutter/foundation.dart';
import 'package:flutter/services.dart';
import 'package:mqtt_client/mqtt_client.dart';
import 'package:mqtt_client/mqtt_server_client.dart';
import '../config/config.dart';

class MQTTService {
  MqttServerClient? _client;
  StreamSubscription<List<MqttReceivedMessage<MqttMessage?>>>?
  _updatesSubscription;

  final Function(String)? onMessageReceived;
  final Function(bool)? onConnectionStatusChanged;
  final Function(String)? onError;

  String _broker = '';
  int _port = 8883;
  String _username = '';
  String _password = '';
  String _deviceId = '';

  bool _isConnecting = false;
  bool _isExplicitDisconnect = false;

  bool get isConnected =>
      _client != null &&
      _client!.connectionStatus!.state == MqttConnectionState.connected;

  bool get isConnecting => _isConnecting;

  MQTTService({
    this.onMessageReceived,
    this.onConnectionStatusChanged,
    this.onError,
  });

  void configure({
    required String broker,
    required int port,
    required String username,
    required String password,
    required String deviceId,
  }) {
    _broker = broker.trim();
    _port = port;
    _username = username.trim();
    _password = password.trim();
    _deviceId = deviceId.trim();
  }

  Future<void> connect({int maxRetries = 3}) async {
    if (isConnected) return;
    if (_isConnecting) return;

    _isConnecting = true;
    _isExplicitDisconnect = false;
    onConnectionStatusChanged?.call(false);

    final sessionIdentifier =
        '${_deviceId}_dimmo_${DateTime.now().millisecondsSinceEpoch % 10000}';

    for (int attempt = 1; attempt <= maxRetries; attempt++) {
      try {
        await _cleanupClient();

        _client = MqttServerClient.withPort(_broker, sessionIdentifier, _port);
        _configureClient();
        await _setupSecurity();
        await _attemptConnection();

        _isConnecting = false;
        return;
      } catch (e) {
        debugPrint('MQTT Connection Attempt $attempt failed: $e');
        if (attempt == maxRetries) {
          _isConnecting = false;
          onError?.call('Connection failed after $maxRetries attempts: $e');
          rethrow;
        }
        await Future.delayed(Duration(seconds: attempt * 2));
      }
    }
    _isConnecting = false;
  }

  void _configureClient() {
    _client!
      ..logging(on: false)
      ..keepAlivePeriod = 60
      ..onConnected = _onConnected
      ..onDisconnected = _onDisconnected
      ..onSubscribed = _onSubscribed
      ..onUnsubscribed = _onUnsubscribed
      ..pongCallback = _onPong
      ..secure = true
      ..autoReconnect = true
      ..resubscribeOnAutoReconnect = true
      ..onAutoReconnect = _onAutoReconnect
      ..onAutoReconnected = _onAutoReconnected
      ..setProtocolV311();
  }

  Future<void> _setupSecurity() async {
    try {
      final context = SecurityContext();
      final certData = await rootBundle.load(sslCertificatePath);
      context.setTrustedCertificatesBytes(certData.buffer.asUint8List());
      _client!.securityContext = context;
      _client!.onBadCertificate = (dynamic cert) => kDebugMode;
    } catch (e) {
      debugPrint('SSL Setup Error: $e');
      throw Exception(
        'Certificate load failed. Verify assets/certificates/client.crt',
      );
    }
  }

  Future<void> _attemptConnection() async {
    final connMess = MqttConnectMessage()
        .withClientIdentifier(_client!.clientIdentifier)
        .authenticateAs(_username, _password)
        .withWillTopic('devices/$_deviceId/status')
        .withWillMessage('offline')
        .startClean()
        .withWillQos(MqttQos.atLeastOnce);

    _client!.connectionMessage = connMess;

    try {
      await _client!.connect();
      if (_client!.connectionStatus!.state != MqttConnectionState.connected) {
        throw Exception('Status: ${_client!.connectionStatus!.state}');
      }
      _setupMessageListener();
    } on Exception catch (e) {
      await _cleanupClient();
      throw Exception('MQTT handshake failed: $e');
    }
  }

  void _setupMessageListener() {
    _updatesSubscription?.cancel();
    if (_client?.updates != null) {
      _updatesSubscription = _client!.updates!.listen(
        (List<MqttReceivedMessage<MqttMessage?>> messages) {
          if (messages.isNotEmpty) {
            final recMsg = messages[0].payload as MqttPublishMessage;
            final payload = MqttPublishPayload.bytesToStringAsString(
              recMsg.payload.message,
            );
            onMessageReceived?.call(payload);
          }
        },
        onError: (error) {
          onError?.call('Stream error: $error');
        },
      );
    }
  }

  Future<void> _cleanupClient() async {
    await _updatesSubscription?.cancel();
    _updatesSubscription = null;
    if (_client != null) {
      try {
        _client!.disconnect();
      } catch (_) {}
      _client = null;
    }
  }

  Future<void> disconnect() async {
    _isExplicitDisconnect = true;
    _isConnecting = false;
    await _cleanupClient();
    onConnectionStatusChanged?.call(false);
  }

  Future<void> subscribe(
    String topic, {
    MqttQos qos = MqttQos.atLeastOnce,
  }) async {
    if (_client == null || !isConnected) return;
    _client!.subscribe(topic, qos);
  }

  Future<void> publish(
    String topic,
    String message, {
    MqttQos qos = MqttQos.atLeastOnce,
  }) async {
    if (_client == null || !isConnected) return;
    final builder = MqttClientPayloadBuilder();
    builder.addString(message);
    _client!.publishMessage(topic, qos, builder.payload!);
  }

  void _onConnected() {
    _isConnecting = false;
    onConnectionStatusChanged?.call(true);
  }

  void _onDisconnected() {
    if (!_isExplicitDisconnect) {
      onConnectionStatusChanged?.call(false);
    }
  }

  void _onAutoReconnect() {
    debugPrint('MQTT Auto Reconnecting...');
    onConnectionStatusChanged?.call(false);
  }

  void _onAutoReconnected() {
    debugPrint('MQTT Auto Reconnected successfully');
    onConnectionStatusChanged?.call(true);
  }

  void _onSubscribed(String topic) {}
  void _onUnsubscribed(String? topic) {}
  void _onPong() {}
}

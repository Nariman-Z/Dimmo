import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:google_fonts/google_fonts.dart';
import '../constants/app_colors.dart';
import '../localization/app_translations.dart';

// --- Theme & Language Manager ---
class ThemeNotifier extends ChangeNotifier {
  static final ThemeNotifier _instance = ThemeNotifier._internal();
  factory ThemeNotifier() => _instance;
  ThemeNotifier._internal();

  bool _isDark = true;
  bool get isDark => _isDark;

  ThemeMode _themeMode = ThemeMode.dark;
  ThemeMode get themeMode => _themeMode;

  String _currentLanguage = 'English';
  String get currentLanguage => _currentLanguage;

  bool get isRtl => _currentLanguage == 'Persian';

  Locale get locale =>
      isRtl ? const Locale('fa', 'IR') : const Locale('en', 'US');

  void setThemeMode(ThemeMode mode) {
    _themeMode = mode;
    if (mode == ThemeMode.dark) {
      _isDark = true;
    } else if (mode == ThemeMode.light) {
      _isDark = false;
    } else {
      final brightness =
          WidgetsBinding.instance.platformDispatcher.platformBrightness;
      _isDark = brightness == Brightness.dark;
    }
    notifyListeners();
  }

  void toggleTheme() {
    if (_isDark) {
      setThemeMode(ThemeMode.light);
    } else {
      setThemeMode(ThemeMode.dark);
    }
  }

  void setLanguage(String lang) {
    _currentLanguage = lang;
    notifyListeners();
  }

  void toggleLanguage() {
    if (_currentLanguage == 'English') {
      _currentLanguage = 'Persian';
    } else {
      _currentLanguage = 'English';
    }
    notifyListeners();
  }

  String translate(String key) {
    return AppTranslations.localizedValues[_currentLanguage]?[key] ?? key;
  }

  String formatNumber(dynamic number) {
    if (number == null) return '';
    String input = number.toString();
    if (!isRtl) return input;

    const english = ['0', '1', '2', '3', '4', '5', '6', '7', '8', '9'];
    const persian = ['۰', '۱', '۲', '۳', '۴', '۵', '۶', '۷', '۸', '۹'];

    for (int i = 0; i < english.length; i++) {
      input = input.replaceAll(english[i], persian[i]);
    }
    return input;
  }

  // --- Dynamic Colors ---
  Color get primaryColor =>
      _isDark ? AppColors.darkPrimary : AppColors.lightPrimary;
  Color get secondaryColor =>
      _isDark ? const Color(0x662A2E3D) : AppColors.lightSecondary;

  Color get textColor =>
      _isDark ? AppColors.darkTextPrimary : AppColors.lightTextPrimary;
  Color get subTextColor =>
      _isDark ? AppColors.darkTextSecondary : AppColors.lightTextSecondary;
  Color get iconColor => _isDark ? Colors.white : const Color(0xFF5A5A5A);

  int get depth => _isDark ? 20 : 10;
}

// --- Static Colors ---
const Color activeColor1 = AppColors.activeOrange;
const Color activeColor2 = AppColors.activeRed;
const List<Color> gradientColors = AppColors.orangeGradient;

const Color greenPrimaryColor = Color(0xFF2E462A);
const Color greenSecondaryColor = Color(0xFF3B5B3A);

// --- Icons ---
const double kMenuIconSize = 24.0;
const double kPowerIconSize = 24.0;
const double kCardIconSize = 32.0;
const double kLightbulbSize = 150.0;
const double kSensorIconSize = 20.0;
const double kDrawerIconSize = 22.0;

// --- Assets ---
const String sslCertificatePath = 'assets/certificates/client.crt';

// --- Enums ---
enum MQTTConnectionStatus {
  connected,
  disconnected,
  connecting,
  error;

  String get name {
    switch (this) {
      case MQTTConnectionStatus.connected:
        return 'Connected';
      case MQTTConnectionStatus.disconnected:
        return 'Disconnected';
      case MQTTConnectionStatus.connecting:
        return 'Connecting...';
      case MQTTConnectionStatus.error:
        return 'Error';
    }
  }
}

// --- App Theme ---
ThemeData getAppTheme(bool isDark, {bool isRtl = false}) {
  final primary = isDark ? AppColors.darkPrimary : AppColors.lightPrimary;
  final baseTextTheme = isDark
      ? ThemeData.dark().textTheme
      : ThemeData.light().textTheme;

  final textTheme = isRtl
      ? GoogleFonts.vazirmatnTextTheme(baseTextTheme)
      : GoogleFonts.poppinsTextTheme(baseTextTheme);

  final overlayStyle = SystemUiOverlayStyle(
    statusBarColor: Colors.transparent,
    statusBarIconBrightness: isDark ? Brightness.light : Brightness.dark,
    statusBarBrightness: isDark ? Brightness.dark : Brightness.light,
    systemNavigationBarColor: primary,
    systemNavigationBarIconBrightness: isDark
        ? Brightness.light
        : Brightness.dark,
    systemNavigationBarDividerColor: Colors.transparent,
    systemNavigationBarContrastEnforced: false,
  );

  return ThemeData(
    brightness: isDark ? Brightness.dark : Brightness.light,
    primaryColor: primary,
    scaffoldBackgroundColor: primary,
    useMaterial3: true,
    textTheme: textTheme,
    appBarTheme: AppBarTheme(
      backgroundColor: Colors.transparent,
      elevation: 0,
      scrolledUnderElevation: 0,
      surfaceTintColor: Colors.transparent,
      centerTitle: true,
      systemOverlayStyle: overlayStyle,
    ),
    colorScheme: ColorScheme.fromSeed(
      seedColor: activeColor1,
      brightness: isDark ? Brightness.dark : Brightness.light,
      primary: activeColor1,
      secondary: activeColor2,
    ),
    sliderTheme: SliderThemeData(
      activeTrackColor: activeColor2,
      inactiveTrackColor: isDark
          ? Colors.black
          : Colors.grey.withValues(alpha: 0.3),
      thumbColor: activeColor1,
    ),
  );
}

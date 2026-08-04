import 'package:flutter/material.dart';

/// Centralized Design Tokens for Dimmo matching flutter_app design guidelines.
class AppColors {
  // Brand & Accent Colors
  static const Color activeOrange = Color(0XFFED560D);
  static const Color activeRed = Color(0XFFCD2A12);
  static const Color primaryBlue = Color(0xFF3498DB);
  static const Color dialPink = Color(0xFFE91E63);
  static const Color dialPurple = Color(0xFF8E44AD);

  // Status Colors
  static const Color successGreen = Color(0xFF4CAF50);
  static const Color activeGreen = Color(0xFF4CAF50);
  static const Color errorRed = Color(0xFFFF5252);
  static const Color warningOrange = Color(0xFFFF9100);

  // Theme: Dark Mode (Classic Neumorphism & Modern Dark)
  static const Color darkPrimary = Color(0xFF26282B);
  static const Color darkSecondary = Color(0xFF1E2023);
  static const Color darkCardBg = Color(0xFF2C2F33);
  static const Color darkTextPrimary = Colors.white;
  static const Color darkTextSecondary = Color(0xB3FFFFFF); // White 70%

  // Theme: Light Mode
  static const Color lightPrimary = Color(0xFFE0E5EC);
  static const Color lightSecondary = Color(0xFFFFFFFF);
  static const Color lightCardBg = Color(0xFFF0F3F8);
  static const Color lightTextPrimary = Color(0xFF2C3E50);
  static const Color lightTextSecondary = Color(0xFF7F8C8D);

  // Gradients
  static const List<Color> orangeGradient = [activeOrange, activeRed];
  static const List<Color> dialGradient = [dialPurple, dialPink];
}

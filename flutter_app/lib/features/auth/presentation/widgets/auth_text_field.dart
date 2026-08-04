import 'package:flutter/material.dart';
import '../../../../core/config/config.dart';
import '../../../../core/constants/app_colors.dart';

/// Premium [TextFormField] styled matching flutter_app AuthTextField design
class AuthTextField extends StatelessWidget {
  final ThemeNotifier theme;
  final TextEditingController controller;
  final String label;
  final IconData icon;
  final bool isObscure;
  final bool isNumber;
  final Iterable<String>? autofillHints;
  final VoidCallback? toggleObscure;
  final bool isRequired;

  const AuthTextField({
    super.key,
    required this.theme,
    required this.controller,
    required this.label,
    required this.icon,
    this.isObscure = false,
    this.isNumber = false,
    this.autofillHints,
    this.toggleObscure,
    this.isRequired = false,
  });

  @override
  Widget build(BuildContext context) {
    final isDark = theme.isDark;
    return Container(
      decoration: BoxDecoration(
        color: isDark ? const Color(0xFF2C2C2E) : Colors.white,
        borderRadius: BorderRadius.circular(16),
        boxShadow: [
          BoxShadow(
            color: Colors.black.withValues(alpha: isDark ? 0.2 : 0.05),
            blurRadius: 10,
            offset: const Offset(0, 4),
          ),
        ],
      ),
      child: TextFormField(
        controller: controller,
        obscureText: isObscure,
        keyboardType: isNumber ? TextInputType.number : TextInputType.text,
        autofillHints: autofillHints,
        style: TextStyle(color: theme.textColor, fontSize: 16),
        decoration: InputDecoration(
          labelText: label,
          labelStyle: TextStyle(
            color: theme.subTextColor.withValues(alpha: 0.6),
            fontSize: 14,
          ),
          floatingLabelStyle: TextStyle(
            color: theme.textColor.withValues(alpha: 0.8),
            fontSize: 12,
            fontWeight: FontWeight.bold,
          ),
          border: OutlineInputBorder(
            borderRadius: BorderRadius.circular(16),
            borderSide: BorderSide.none,
          ),
          focusedBorder: OutlineInputBorder(
            borderRadius: BorderRadius.circular(16),
            borderSide: BorderSide.none,
          ),
          enabledBorder: OutlineInputBorder(
            borderRadius: BorderRadius.circular(16),
            borderSide: BorderSide.none,
          ),
          errorBorder: OutlineInputBorder(
            borderRadius: BorderRadius.circular(16),
            borderSide: const BorderSide(color: AppColors.errorRed, width: 1),
          ),
          focusedErrorBorder: OutlineInputBorder(
            borderRadius: BorderRadius.circular(16),
            borderSide: const BorderSide(color: AppColors.errorRed, width: 2),
          ),
          errorStyle: const TextStyle(
            color: AppColors.errorRed,
            fontSize: 12,
            fontWeight: FontWeight.w500,
          ),
          prefixIcon: Icon(icon, color: theme.subTextColor, size: 20),
          suffixIcon: toggleObscure != null
              ? IconButton(
                  icon: Icon(
                    isObscure
                        ? Icons.visibility_off_outlined
                        : Icons.visibility_outlined,
                    color: theme.subTextColor,
                    size: 20,
                  ),
                  onPressed: toggleObscure,
                )
              : null,
          contentPadding: const EdgeInsets.symmetric(
            horizontal: 20,
            vertical: 20,
          ),
          floatingLabelBehavior: FloatingLabelBehavior.auto,
        ),
        validator: (v) {
          if (isRequired && (v == null || v.isEmpty)) {
            return theme.translate('required_field');
          }
          return null;
        },
      ),
    );
  }
}

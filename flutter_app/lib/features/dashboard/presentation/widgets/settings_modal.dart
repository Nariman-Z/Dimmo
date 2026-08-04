import 'package:flutter/material.dart';
import 'package:url_launcher/url_launcher.dart';
import '../../../../core/config/config.dart';
import '../../../../core/constants/app_colors.dart';

/// A modal dialog for application settings and device information.
class SettingsModal extends StatelessWidget {
  final ThemeNotifier theme;
  final String deviceId;
  final String brokerAddress;
  final int port;
  final String username;
  final MQTTConnectionStatus connectionStatus;
  final VoidCallback onLogout;

  const SettingsModal({
    super.key,
    required this.theme,
    required this.deviceId,
    required this.brokerAddress,
    required this.port,
    required this.username,
    required this.connectionStatus,
    required this.onLogout,
  });

  Future<void> _launchGithub(BuildContext context) async {
    final Uri url = Uri.parse('https://github.com/Nariman-Z');
    if (!await launchUrl(url, mode: LaunchMode.externalApplication)) {
      if (context.mounted) {
        ScaffoldMessenger.of(context).showSnackBar(
          const SnackBar(content: Text('Could not open GitHub link')),
        );
      }
    }
  }

  @override
  Widget build(BuildContext context) {
    final isDark = theme.isDark;
    final textColor = theme.textColor;
    final isConnected = connectionStatus == MQTTConnectionStatus.connected;

    return Dialog(
      shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(24)),
      backgroundColor: theme.primaryColor,
      surfaceTintColor: Colors.transparent,
      insetPadding: const EdgeInsets.symmetric(horizontal: 20, vertical: 24),
      child: Container(
        constraints: BoxConstraints(
          maxHeight: MediaQuery.of(context).size.height * 0.85,
          maxWidth: 500,
        ),
        padding: const EdgeInsets.all(24.0),
        child: Column(
          mainAxisSize: MainAxisSize.min,
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            // Modal Title & Close Button
            Row(
              mainAxisAlignment: MainAxisAlignment.spaceBetween,
              children: [
                Text(
                  theme.translate('settings_title'),
                  style: TextStyle(
                    fontSize: 24,
                    fontWeight: FontWeight.bold,
                    color: textColor,
                  ),
                ),
                IconButton(
                  onPressed: () => Navigator.of(context).pop(),
                  icon: Icon(
                    Icons.close,
                    color: textColor.withValues(alpha: 0.6),
                  ),
                  style: IconButton.styleFrom(
                    backgroundColor: isDark
                        ? Colors.white10
                        : Colors.black.withValues(alpha: 0.05),
                    padding: const EdgeInsets.all(8),
                  ),
                ),
              ],
            ),
            const SizedBox(height: 16),

            // Scrollable Content
            Flexible(
              child: SingleChildScrollView(
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    // --- Section 1: APPEARANCE (At the top) ---
                    _SectionHeader(
                      theme: theme,
                      title: theme.translate('appearance'),
                    ),
                    const SizedBox(height: 12),
                    Row(
                      children: [
                        _ThemeCard(
                          theme: theme,
                          mode: ThemeMode.light,
                          label: theme.translate('light'),
                          icon: Icons.wb_sunny_outlined,
                          isSelected: theme.themeMode == ThemeMode.light,
                          onTap: () => theme.setThemeMode(ThemeMode.light),
                        ),
                        const SizedBox(width: 12),
                        _ThemeCard(
                          theme: theme,
                          mode: ThemeMode.dark,
                          label: theme.translate('dark'),
                          icon: Icons.nightlight_round,
                          isSelected: theme.themeMode == ThemeMode.dark,
                          onTap: () => theme.setThemeMode(ThemeMode.dark),
                        ),
                        const SizedBox(width: 12),
                        _ThemeCard(
                          theme: theme,
                          mode: ThemeMode.system,
                          label: theme.translate('system'),
                          icon: Icons.devices_rounded,
                          isSelected: theme.themeMode == ThemeMode.system,
                          onTap: () => theme.setThemeMode(ThemeMode.system),
                        ),
                      ],
                    ),

                    const SizedBox(height: 16),
                    Divider(color: textColor.withValues(alpha: 0.1), height: 1),
                    const SizedBox(height: 16),

                    // --- Section 2: DEVICE STATUS ---
                    _SectionHeader(
                      theme: theme,
                      title: theme.translate('device_status'),
                    ),
                    const SizedBox(height: 12),
                    _StatusRow(
                      theme: theme,
                      label: theme.translate('status'),
                      value: isConnected
                          ? theme.translate('online')
                          : theme.translate('offline'),
                      statusColor: isConnected
                          ? AppColors.successGreen
                          : AppColors.errorRed,
                    ),

                    const SizedBox(height: 16),
                    Divider(color: textColor.withValues(alpha: 0.1), height: 1),
                    const SizedBox(height: 16),

                    // --- Section 3: BROKER & DEVICE INFO ---
                    _SectionHeader(
                      theme: theme,
                      title: theme.translate('broker_info'),
                    ),
                    const SizedBox(height: 12),
                    _InfoRow(
                      theme: theme,
                      label: theme.translate('device_id_label'),
                      value: deviceId,
                    ),
                    const SizedBox(height: 8),
                    _InfoRow(
                      theme: theme,
                      label: theme.translate('broker_address'),
                      value: brokerAddress,
                    ),
                    const SizedBox(height: 8),
                    _InfoRow(
                      theme: theme,
                      label: theme.translate('port_label'),
                      value: port.toString(),
                    ),
                    if (username.isNotEmpty) ...[
                      const SizedBox(height: 8),
                      _InfoRow(
                        theme: theme,
                        label: theme.translate('username_optional'),
                        value: username,
                      ),
                    ],

                    const SizedBox(height: 16),
                    Divider(color: textColor.withValues(alpha: 0.1), height: 1),
                    const SizedBox(height: 16),

                    // --- Section 4: LANGUAGE ---
                    _SectionHeader(
                      theme: theme,
                      title: theme.translate('language'),
                    ),
                    const SizedBox(height: 12),
                    _LanguageOptionTile(
                      theme: theme,
                      label: theme.translate('english'),
                      isSelected: theme.currentLanguage == 'English',
                      onTap: () => theme.setLanguage('English'),
                    ),
                    const SizedBox(height: 8),
                    _LanguageOptionTile(
                      theme: theme,
                      label: theme.translate('persian'),
                      isSelected: theme.currentLanguage == 'Persian',
                      onTap: () => theme.setLanguage('Persian'),
                    ),

                    const SizedBox(height: 20),
                    Divider(color: textColor.withValues(alpha: 0.1), height: 1),
                    const SizedBox(height: 20),

                    // Logout & Exit Action
                    SizedBox(
                      width: double.infinity,
                      child: ElevatedButton.icon(
                        onPressed: () {
                          Navigator.of(context).pop();
                          onLogout();
                        },
                        icon: const Icon(Icons.logout_rounded, size: 20),
                        label: Text(
                          theme.translate('logout_exit'),
                          style: const TextStyle(
                            fontSize: 15,
                            fontWeight: FontWeight.bold,
                          ),
                        ),
                        style: ElevatedButton.styleFrom(
                          backgroundColor: AppColors.errorRed.withValues(
                            alpha: 0.1,
                          ),
                          foregroundColor: AppColors.errorRed,
                          elevation: 0,
                          padding: const EdgeInsets.symmetric(vertical: 14),
                          shape: RoundedRectangleBorder(
                            borderRadius: BorderRadius.circular(16),
                            side: BorderSide(
                              color: AppColors.errorRed.withValues(alpha: 0.3),
                            ),
                          ),
                        ),
                      ),
                    ),

                    const SizedBox(height: 20),

                    // Footer with GitHub link redirection
                    Center(
                      child: InkWell(
                        onTap: () => _launchGithub(context),
                        borderRadius: BorderRadius.circular(12),
                        child: Container(
                          padding: const EdgeInsets.symmetric(
                            horizontal: 16,
                            vertical: 10,
                          ),
                          decoration: BoxDecoration(
                            color: textColor.withValues(alpha: 0.04),
                            borderRadius: BorderRadius.circular(12),
                          ),
                          child: Row(
                            mainAxisSize: MainAxisSize.min,
                            children: [
                              Text(
                                "Made with ♥ by Nariman_Z",
                                style: TextStyle(
                                  color: theme.subTextColor,
                                  fontSize: 13,
                                  fontWeight: FontWeight.w500,
                                ),
                              ),
                              const SizedBox(width: 6),
                              Icon(
                                Icons.open_in_new_rounded,
                                size: 14,
                                color: theme.subTextColor,
                              ),
                            ],
                          ),
                        ),
                      ),
                    ),
                  ],
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }
}

class _ThemeCard extends StatelessWidget {
  final ThemeNotifier theme;
  final ThemeMode mode;
  final String label;
  final IconData icon;
  final bool isSelected;
  final VoidCallback onTap;

  const _ThemeCard({
    required this.theme,
    required this.mode,
    required this.label,
    required this.icon,
    required this.isSelected,
    required this.onTap,
  });

  @override
  Widget build(BuildContext context) {
    final isDark = theme.isDark;
    final activeColor = AppColors.activeOrange;
    final borderColor = isSelected
        ? activeColor
        : (isDark ? Colors.white12 : Colors.black12);
    final iconColor = isSelected
        ? activeColor
        : theme.textColor.withValues(alpha: 0.7);
    final textColor = isSelected
        ? activeColor
        : theme.textColor.withValues(alpha: 0.7);
    final bgColor = isSelected
        ? activeColor.withValues(alpha: 0.1)
        : (isDark
              ? Colors.white.withValues(alpha: 0.04)
              : Colors.black.withValues(alpha: 0.03));

    return Expanded(
      child: Material(
        color: Colors.transparent,
        child: InkWell(
          onTap: onTap,
          borderRadius: BorderRadius.circular(16),
          child: Container(
            padding: const EdgeInsets.symmetric(vertical: 16),
            decoration: BoxDecoration(
              color: bgColor,
              borderRadius: BorderRadius.circular(16),
              border: Border.all(
                color: borderColor,
                width: isSelected ? 2.0 : 1.0,
              ),
            ),
            child: Column(
              mainAxisSize: MainAxisSize.min,
              children: [
                Icon(icon, color: iconColor, size: 26),
                const SizedBox(height: 8),
                Text(
                  label,
                  style: TextStyle(
                    color: textColor,
                    fontSize: 13,
                    fontWeight: isSelected ? FontWeight.bold : FontWeight.w500,
                  ),
                ),
              ],
            ),
          ),
        ),
      ),
    );
  }
}

class _LanguageOptionTile extends StatelessWidget {
  final ThemeNotifier theme;
  final String label;
  final bool isSelected;
  final VoidCallback onTap;

  const _LanguageOptionTile({
    required this.theme,
    required this.label,
    required this.isSelected,
    required this.onTap,
  });

  @override
  Widget build(BuildContext context) {
    final textColor = theme.textColor;
    final activeColor = AppColors.activeOrange;

    return Material(
      color: Colors.transparent,
      child: InkWell(
        onTap: onTap,
        borderRadius: BorderRadius.circular(14),
        child: Container(
          padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 12),
          decoration: BoxDecoration(
            color: isSelected
                ? activeColor.withValues(alpha: 0.1)
                : textColor.withValues(alpha: 0.04),
            borderRadius: BorderRadius.circular(14),
            border: Border.all(
              color: isSelected
                  ? activeColor
                  : textColor.withValues(alpha: 0.1),
              width: isSelected ? 1.5 : 1.0,
            ),
          ),
          child: Row(
            mainAxisAlignment: MainAxisAlignment.spaceBetween,
            children: [
              Text(
                label,
                style: TextStyle(
                  color: isSelected ? activeColor : textColor,
                  fontSize: 15,
                  fontWeight: isSelected ? FontWeight.bold : FontWeight.w500,
                ),
              ),
              if (isSelected)
                Icon(Icons.check_rounded, color: activeColor, size: 20),
            ],
          ),
        ),
      ),
    );
  }
}

class _SectionHeader extends StatelessWidget {
  final ThemeNotifier theme;
  final String title;
  const _SectionHeader({required this.theme, required this.title});

  @override
  Widget build(BuildContext context) {
    return Text(
      title,
      style: TextStyle(
        fontSize: 12,
        fontWeight: FontWeight.bold,
        color: theme.textColor.withValues(alpha: 0.5),
        letterSpacing: 1.2,
      ),
    );
  }
}

class _StatusRow extends StatelessWidget {
  final ThemeNotifier theme;
  final String label;
  final String value;
  final Color statusColor;

  const _StatusRow({
    required this.theme,
    required this.label,
    required this.value,
    required this.statusColor,
  });

  @override
  Widget build(BuildContext context) {
    final textColor = theme.textColor;
    return Container(
      padding: const EdgeInsets.all(12),
      decoration: BoxDecoration(
        color: textColor.withValues(alpha: 0.05),
        borderRadius: BorderRadius.circular(12),
      ),
      child: Row(
        mainAxisAlignment: MainAxisAlignment.spaceBetween,
        children: [
          Text(
            label,
            style: TextStyle(
              color: textColor,
              fontSize: 14,
              fontWeight: FontWeight.w500,
            ),
          ),
          Row(
            children: [
              Container(
                width: 8,
                height: 8,
                decoration: BoxDecoration(
                  color: statusColor,
                  shape: BoxShape.circle,
                ),
              ),
              const SizedBox(width: 8),
              Text(
                value,
                style: TextStyle(
                  color: textColor,
                  fontSize: 14,
                  fontWeight: FontWeight.bold,
                ),
              ),
            ],
          ),
        ],
      ),
    );
  }
}

class _InfoRow extends StatelessWidget {
  final ThemeNotifier theme;
  final String label;
  final String value;

  const _InfoRow({
    required this.theme,
    required this.label,
    required this.value,
  });

  @override
  Widget build(BuildContext context) {
    final textColor = theme.textColor;
    return Row(
      mainAxisAlignment: MainAxisAlignment.spaceBetween,
      children: [
        Text(
          label,
          style: TextStyle(
            color: textColor.withValues(alpha: 0.6),
            fontSize: 14,
            fontWeight: FontWeight.w500,
          ),
        ),
        const SizedBox(width: 16),
        Expanded(
          child: Text(
            value,
            textAlign: TextAlign.end,
            overflow: TextOverflow.ellipsis,
            maxLines: 1,
            style: TextStyle(
              color: textColor,
              fontSize: 14,
              fontWeight: FontWeight.bold,
            ),
          ),
        ),
      ],
    );
  }
}

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'core/config/config.dart';
import 'features/auth/presentation/screens/login_screen.dart';

void main() async {
  WidgetsFlutterBinding.ensureInitialized();
  await SystemChrome.setPreferredOrientations([
    DeviceOrientation.portraitUp,
    DeviceOrientation.portraitDown,
  ]);
  runApp(const MyApp());
}

class MyApp extends StatelessWidget {
  const MyApp({super.key});

  @override
  Widget build(BuildContext context) {
    return ListenableBuilder(
      listenable: ThemeNotifier(),
      builder: (context, _) {
        final theme = ThemeNotifier();
        return MaterialApp(
          title: 'Dimmo',
          debugShowCheckedModeBanner: false,
          theme: getAppTheme(theme.isDark, isRtl: theme.isRtl),
          builder: (context, child) {
            return Directionality(
              textDirection: theme.isRtl
                  ? TextDirection.rtl
                  : TextDirection.ltr,
              child: child!,
            );
          },
          home: const LoginPage(),
        );
      },
    );
  }
}

#ifndef TEST_UTILS_H
#define TEST_UTILS_H

#define TEST_CONSOLE_OUTPUT
#ifdef TEST_CONSOLE_OUTPUT
#define CONSOLE_OUTPUT(str) do { std::cerr << "[>>>>>>>>>>] " << str << '\n'; } while  (false)
#else
#define CONSOLE_OUTPUT(str) do { } while (false)
#endif

#endif // TEST_UTILS_H

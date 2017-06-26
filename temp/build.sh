g++ calibrate.cpp -o calibrate `pkg-config --libs opencv` -Wall -Wextra -Wfloat-equal -Wundef -Wcast-align -Wwrite-strings -Wlogical-op -Wmissing-declarations -Wredundant-decls -Wshadow -Woverloaded-virtual -std=c++11 -I .

./calibrate
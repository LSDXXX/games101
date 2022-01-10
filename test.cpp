#include <iostream>
#include <format>
#include <string_view>

struct point {
    int x;
    int y;
    operator std::string_view() {
        return "{" + std::to_string(x) + "," + std::to_string(y) + "}";
    }
}

int main() {
    point p;
    p.x = 10;
    p.y = 20;
    std::format("point: {}", p);
}



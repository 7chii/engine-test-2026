#include "app.h"

int main() {
    App app;
    if (!app.init()) return -1;

    while (!app.shouldClose()) {
        app.frame();
    }

    app.shutdown();
    return 0;
}

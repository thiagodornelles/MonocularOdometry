#include <odometry.h>

int main(int argc, char *argv[]) {
    Odometry odom(argc, argv, true);
    odom.initialize();
    while(1);
    return 0;
}

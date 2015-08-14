//
// Created by Mohit Deshpande on 7/30/15.
//

#include <fstream>
#include <iostream>
#include <random>

using namespace std;

int main(int argc, char *argv[]) {
    int n;
    if (argc < 2) n = 10;
    else n = atoi(argv[1]);

    ofstream fclean("box_clean.txt", ofstream::out);
    ofstream fnoisy("box_noisy.txt", ofstream::out);
    double res = 1.0 / n;

    default_random_engine eng;
    normal_distribution<> dist(0, 1);

    // z = 0 and z = 1
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            // plane: z = 0
            double x = res * i;
            double y = res * j;
            double z = 0;
            fclean << x << " " << y << " " << z << endl;

            double x_noise = dist(eng) * (.10 * res);
            double y_noise = dist(eng) * (.10 * res);
            double z_noise = dist(eng) * (.10 * res);
            fnoisy << x + x_noise << " " << y + y_noise << " " << z + z_noise << endl;

            // plane: z = 1
            z = 1;
            fclean << x << " " << y << " " << z << endl;

            x_noise = dist(eng) * (.10 * res);
            y_noise = dist(eng) * (.10 * res);
            z_noise = dist(eng) * (.10 * res);
            fnoisy << x + x_noise << " " << y + y_noise << " " << z + z_noise << endl;
        }
    }

    // y = 0 and y = 1
    for (int i = 0; i < n; i++) {
        for (int k = 0; k < n; k++) {
            // plane: y = 0
            double x = res * i;
            double y = 0;
            double z = res * k;
            fclean << x << " " << y << " " << z << endl;

            double x_noise = dist(eng) * (.10 * res);
            double y_noise = dist(eng) * (.10 * res);
            double z_noise = dist(eng) * (.10 * res);
            fnoisy << x + x_noise << " " << y + y_noise << " " << z + z_noise << endl;

            // plane: y = 1
            y = 1;
            fclean << x << " " << y << " " << z << endl;

            x_noise = dist(eng) * (.10 * res);
            y_noise = dist(eng) * (.10 * res);
            z_noise = dist(eng) * (.10 * res);
            fnoisy << x + x_noise << " " << y + y_noise << " " << z + z_noise << endl;
        }
    }

    // x = 0 and x = 1
    for (int j = 0; j < n; j++) {
        for (int k = 0; k < n; k++) {
            // plane: y = 0
            double x = 0;
            double y = res * j;
            double z = res * k;
            fclean << x << " " << y << " " << z << endl;

            double x_noise = dist(eng) * (.10 * res);
            double y_noise = dist(eng) * (.10 * res);
            double z_noise = dist(eng) * (.10 * res);
            fnoisy << x + x_noise << " " << y + y_noise << " " << z + z_noise << endl;

            // plane: y = 1
            x = 1;
            fclean << x << " " << y << " " << z << endl;

            x_noise = dist(eng) * (.10 * res);
            y_noise = dist(eng) * (.10 * res);
            z_noise = dist(eng) * (.10 * res);
            fnoisy << x + x_noise << " " << y + y_noise << " " << z + z_noise << endl;
        }
    }

    cout << "Generated " << n * n * 6 << " points" << endl;

    fclean.close();
    fnoisy.close();
}

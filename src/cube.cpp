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

    ofstream fout("box_clean.txt", ofstream::out);
    ofstream fout2("box_noisy.txt", ofstream::out);
    double res = 1.0 / n;

    default_random_engine eng;
    normal_distribution<> dist(0, 1);

    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            for (int k = 0; k < n; k++) {
                double x_noise = dist(eng) * (.10 * res);
                double y_noise = dist(eng) * (.10 * res);
                double z_noise = dist(eng) * (.10 * res);

                // Clean data set
                double x = res * i;
                double y = res * j;
                double z = res * k;
                fout << x << " " << y << " " << z << endl;

                // Noisy data set
                x += x_noise;
                y += y_noise;
                z += z_noise;
                fout2 << x << " " << y << " " << z << endl;
            }
        }
    }

    cout << "Generated " << pow(n, 3) << " points" << endl;

    fout.close();
    fout2.close();
}

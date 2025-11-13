#include "pathsmooth.hpp"
#include <iostream>

int main(int argc, char** argv) {
    std::string in="", out="";
    SmoothParams p;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto need = [&](const char* name){ if (++i>=argc){std::cerr<<"Missing "<<name<<"\n";return std::string();} return std::string(argv[i]); };
        if (a=="--input"||a=="-i") in = need("--input");
        else if (a=="--output"||a=="-o") out = need("--output");
        else if (a=="--num_points"||a=="-n") p.num_points = std::stoi(need("--num_points"));
        else if (a=="--smoothing"||a=="-s") p.smoothing_passes = std::stoi(need("--smoothing"));
        else { std::cerr<<"Unknown arg "<<a<<"\n"; return 2; }
    }
    if (in.empty() || out.empty()) {
        std::cerr << "Usage: pathsmooth_cli --input in.csv --output out.csv [--num_points 300] [--smoothing 4]\n";
        return 2;
    }

    try {
        auto wps = LoadWaypointsCsv(in, /*has_header=*/true);
        auto sm  = SmoothPathBSpline(wps, p);
        SaveCsvXY(out, sm);
        std::cout << "Smoothed path saved to " << out << "\n";
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n"; return 1;
    }
    return 0;
}

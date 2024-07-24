#ifndef MOTION_GENERATORS_HPP
#define MOTION_GENERATORS_HPP

#include <math.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>
#include <stdexcept>
#include <filesystem>

namespace motion_generators
{


const float pulseToRad[8] =
{
    153054.453125,
    153054.453125,
    134496.984375,
    92770.187500,
    92770.187500,
    92770.187500,
    0.000000,
    0.000000
};

const float maxVelocityRad[8] = 
{
    2.26880037078307,
    2.26880037078307,
    3.1413343723900873,
    3.1394784019381228,
    4.362931787757786,
    4.362931787757786,
    0.0,
    0.0,
};

const unsigned int maxIncrement[8] = 
{
    1389,
    1389,
    1690,
    1165,
    1619,
    1619,
    0,
    0
};

// class Generator
// {
// public:
//     // pos0: [rad], v: [rad/sec], a: [rad/sec^2]
//     Generator(double pos0, double v, double a) :
//         _p0(pos0), _vel(v), _acc(a) {}

//     /*
//     t = time [s]
//     p = position [rad]
//     v = velocity [rad / sec]
//     a = acceleration [rad / sec^2]
//     */
//     virtual void get_values(const double &t, double &p, double &v, double &a);

// protected:
//     double _p0, _vel, _acc;
//     //double _p, _v, _a;
// };

class Sine
{
public:
    Sine(double pos0, double v, double a) :
        _p0(pos0), _vel(v), _acc(a)
    {
        
        A = _vel * _vel / _acc;
        B = _acc / _vel;
    }
    // # pos = a*sin(b*x)
    // # vel = a*b*cos(b*x)
    // # acc = -a*b*b*sin(b*x)


    void get_values(const double &t, double &p, double &v, double &a)
    {
        p = A * sin(t * B) + _p0;
        v = A * B * cos(t * B);
        a = -A * B*B * sin(t * B);           
    }

private:
    double A,B;
    double _p0, _vel, _acc;
};

class Ramp
{
public:
    Ramp(double pos0, double v, double a) :
        _p0(pos0), _vel(v), _acc(a)
    {}

    void get_values(const double &t, double &p, double &v, double &a)
    {
        p = _vel * t + _p0;
        v = _vel;
        a = 0;
    }
private:
    double _p0, _vel, _acc;

};

class Step {
public:
    Step(double pos) : _p(pos) {}
    void get_values(const double &/*t*/, double &p, double &/*v*/, double &/*a*/)
    { p = _p; }

private:
    double _p;
};

class CSVreader
{
public:
    CSVreader(std::string & path)
    {
        parse(path);
        // for(auto v : values)
        // std::cout << "values: " << v << std::endl;
    }

    std::vector<double> values;

private:

    void parse(std::string & _path)
    {
        // Parse CSV file
        
        if(!std::filesystem::exists(_path))
        {
            throw std::runtime_error("[CSV] file does not exits");
        }

        std::string line, word;
        std::ifstream ifs (_path);
        // auto line_count = std::count(std::istreambuf_iterator<char>(ifs), 
        //     std::istreambuf_iterator<char>(), '\n');
        // std::cout << "lines: " << line_count << '\n';

        if(ifs.is_open())
        {
            // Get lines
            while(std::getline(ifs, line, ';'))
            {
                if(ifs.fail()) break;
                if(line.length() < 2) continue;

                std::stringstream s(line);
                
                // Get words
                while(std::getline(s, word, ','))
                {
                    // Convert to double
                    values.push_back(strtod(word.c_str(), nullptr));
                }
            }

        }
        if(!values.size()) 
        {
            throw std::runtime_error("[CSV] no values");
        }


    }
};

class RealData
{
public:
    RealData(std::string file_path) : _csv(file_path)
    {}

    void get_values(const double &/*t*/, double &p, double & /*v*/, double &/*a*/)
    {
        if(value_counter < _csv.values.size())
        {
            p = _csv.values[value_counter++];
        }
        else
        {
            p = _csv.values.back();
            std::cout << "[REAL_DATA] end of values" << std::endl;
        }
    }
private:
    CSVreader _csv;
    size_t value_counter = 0;
};





} // motion_generators


#endif // MOTION_GENERATORS_HPP

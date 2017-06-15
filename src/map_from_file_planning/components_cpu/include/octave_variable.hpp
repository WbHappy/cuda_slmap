#ifndef TEMPLATE_OCTAVE_VARIABLE_HPP_
#define TEMPLATE_OCTAVE_VARIABLE_HPP_

#include <cstring>
#include <cmath>

template <typename data_type>
class OctaveVariable{
public:
    data_type data;

public:

    OctaveVariable(){
    }

    OctaveVariable(data_type data){
        this->data = data;
    }

    ~OctaveVariable(){};

// Minus operator

    OctaveVariable operator-(){
        OctaveVariable result (-1 * this->data);
        return result;
    }
};

// Class & Class argument operators
    template <typename data_type>
    OctaveVariable <data_type> operator+(const OctaveVariable<data_type>& mv1, const OctaveVariable<data_type>& mv2){
        OctaveVariable <data_type> result (mv1.data + mv2.data);
        return result;
    }

    template <typename data_type>
    OctaveVariable <data_type> operator-(const OctaveVariable<data_type>& mv1, const OctaveVariable<data_type>& mv2){
        OctaveVariable <data_type> result (mv1.data - mv2.data);
        return result;
    }

    template <typename data_type>
    OctaveVariable <data_type> operator*(const OctaveVariable<data_type>& mv1, const OctaveVariable<data_type>& mv2){
        OctaveVariable <data_type> result (mv1.data * mv2.data);
        return result;
    }

    template <typename data_type>
    OctaveVariable <data_type> operator/(const OctaveVariable<data_type>& mv1, const OctaveVariable<data_type>& mv2){
        OctaveVariable <data_type> result (mv1.data / mv2.data);
        return result;
    }

    template <typename data_type>
    OctaveVariable <data_type> operator->*(const OctaveVariable<data_type>& mv1, const OctaveVariable<data_type>& mv2){
        OctaveVariable <data_type> result ( pow(mv1.data, mv2.data) );
        return result;
    }

// 
// // Class & Int argument operators
//     template <typename data_type>
//     OctaveVariable <data_type> operator+(const OctaveVariable<data_type>& mv, const int& dt){
//         OctaveVariable <data_type> result (mv.data + (data_type)dt);
//         return result;
//     }
//
//     template <typename data_type>
//     OctaveVariable <data_type> operator-(const OctaveVariable<data_type>& mv, const int& dt){
//         OctaveVariable <data_type> result (mv.data - (data_type)dt);
//         return result;
//     }
//
//     template <typename data_type>
//     OctaveVariable <data_type> operator*(const OctaveVariable<data_type>& mv, const int& dt){
//         OctaveVariable <data_type> result (mv.data * (data_type)dt);
//         return result;
//     }
//
//     template <typename data_type>
//     OctaveVariable <data_type> operator/(const OctaveVariable<data_type>& mv, const int& dt){
//         OctaveVariable <data_type> result (mv.data / (data_type)dt);
//         return result;
//     }
//
//     template <typename data_type>
//     OctaveVariable <data_type> operator->*(const OctaveVariable<data_type>& mv, const int& dt){
//         OctaveVariable <data_type> result ( pow(mv.data, (data_type)dt) );
//         return result;
//     }
//
// // Int & Class argument operators
//     template <typename data_type>
//     OctaveVariable <data_type> operator+(const int& dt, const OctaveVariable<data_type>& mv){
//         OctaveVariable <data_type> result ((data_type)dt + mv.data);
//         return result;
//     }
//
//     template <typename data_type>
//     OctaveVariable <data_type> operator-(const int& dt, const OctaveVariable<data_type>& mv){
//         OctaveVariable <data_type> result ((data_type)dt - mv.data);
//         return result;
//     }
//
//     template <typename data_type>
//     OctaveVariable <data_type> operator*(const int& dt, const OctaveVariable<data_type>& mv){
//         OctaveVariable <data_type> result ((data_type)dt * mv.data);
//         return result;
//     }
//
//     template <typename data_type>
//     OctaveVariable <data_type> operator/(const int& dt, const OctaveVariable<data_type>& mv){
//         OctaveVariable <data_type> result ((data_type)dt / mv.data);
//         return result;
//     }
//
//     template <typename data_type>
//     OctaveVariable <data_type> operator->*(const int& dt, const OctaveVariable<data_type>& mv){
//         OctaveVariable <data_type> result ( pow((data_type)dt, mv.data) );
//         return result;
//     }

// Class & Double argument operators
    template <typename data_type>
    OctaveVariable <data_type> operator+(const OctaveVariable<data_type>& mv, const double& dt){
        OctaveVariable <data_type> result (mv.data + dt);
        return result;
    }

    template <typename data_type>
    OctaveVariable <data_type> operator-(const OctaveVariable<data_type>& mv, const double& dt){
        OctaveVariable <data_type> result (mv.data - dt);
        return result;
    }

    template <typename data_type>
    OctaveVariable <data_type> operator*(const OctaveVariable<data_type>& mv, const double& dt){
        OctaveVariable <data_type> result (mv.data * dt);
        return result;
    }

    template <typename data_type>
    OctaveVariable <data_type> operator/(const OctaveVariable<data_type>& mv, const double& dt){
        OctaveVariable <data_type> result (mv.data / dt);
        return result;
    }

    template <typename data_type>
    OctaveVariable <data_type> operator->*(const OctaveVariable<data_type>& mv, const double& dt){
        OctaveVariable <data_type> result ( pow(mv.data, dt) );
        return result;
    }

// Double & Class argument operators
    template <typename data_type>
    OctaveVariable <data_type> operator+(const double& dt, const OctaveVariable<data_type>& mv){
        OctaveVariable <data_type> result (dt + mv.data);
        return result;
    }

    template <typename data_type>
    OctaveVariable <data_type> operator-(const double& dt, const OctaveVariable<data_type>& mv){
        OctaveVariable <data_type> result (dt - mv.data);
        return result;
    }

    template <typename data_type>
    OctaveVariable <data_type> operator*(const double& dt, const OctaveVariable<data_type>& mv){
        OctaveVariable <data_type> result (dt * mv.data);
        return result;
    }

    template <typename data_type>
    OctaveVariable <data_type> operator/(const double& dt, const OctaveVariable<data_type>& mv){
        OctaveVariable <data_type> result (dt / mv.data);
        return result;
    }

    template <typename data_type>
    OctaveVariable <data_type> operator->*(const double& dt, const OctaveVariable<data_type>& mv){
        OctaveVariable <data_type> result ( pow(dt, mv.data) );
        return result;
    }

// Trigonometric functions
    template <typename data_type>
    OctaveVariable <data_type> sin(OctaveVariable <data_type> input){
        OctaveVariable <data_type> result;
        result.data = sin(input.data);
        return result;
    }

    template <typename data_type>
    OctaveVariable <data_type> cos(OctaveVariable <data_type> input){
        OctaveVariable <data_type> result;
        result.data = cos(input.data);
        return result;
    }

    template <typename data_type>
    OctaveVariable <data_type> tan(OctaveVariable <data_type> input){
        OctaveVariable <data_type> result;
        result.data = tan(input.data);
        return result;
    }

    template <typename data_type>
    OctaveVariable <data_type> atan(OctaveVariable <data_type> input){
        OctaveVariable <data_type> result;
        result.data = atan(input.data);
        return result;
    }

    template <typename data_type>
    OctaveVariable <data_type> atan2(OctaveVariable <data_type> input1, OctaveVariable <data_type> input2){
        OctaveVariable <data_type> result;
        result.data = atan2(input1.data, input2.data);
        return result;
    }


#endif

#ifndef LOWPASSFILTER_H
#define LOWPASSFILTER_H

#include <iostream>
#include <cmath>

namespace quatlib
{
    using namespace std;

    template <typename T>
    class LowpassFilter
    {
    public:
        LowpassFilter(): coeff(0.2), filterenable(false) {};
        LowpassFilter(float ccoef, bool filten): coeff(ccoef), filterenable(filten) {};
        ~LowpassFilter() {};

        void setfilenable(float flag);
        void setcutofffreq(float cutOffFreq, float loopper);
        T update(T s);

    private:
        float coeff;
        bool filterenable;

        T s_1;
    };

    template<typename T>
    void LowpassFilter<T>::setfilenable(float flag)
    {
        filterenable = flag;
    }

    template<typename T>
    void LowpassFilter<T>::setcutofffreq(float cutOffFreq, float loopper)
    {
		    coeff = 1 - exp(-2 * M_PI * cutOffFreq * loopper);
	  }    

    template<typename T>
    T LowpassFilter<T>::update(T s)
    {
        if (!filterenable)
        {
            s_1 = s;
        }
        else
        {
            s_1 = s_1*(1 - coeff) + s*coeff;
        }

        return s_1;
    }
}

#endif // LOWPASSFILTER_H

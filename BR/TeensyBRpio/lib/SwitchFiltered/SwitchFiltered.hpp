#ifndef __H_SWITCHS
#define __H_SWITCHS

#include <FilterLowPass.h>

class SwitchFiltered
{
public :
    SwitchFiltered(int pinSwitch);
    bool isSwitchPressed() const;
    void loop();
protected :
    const int SWITCH_PIN;
    static constexpr float PERIOD_FILTER_SWITCH = 0.05; //s
    static constexpr float SEUIL_SWITCH = 0.7; // static constexpr est une constante partagée à toutes les instances de la classe, sa valeur est connue à la compilation
                                               // avec constexpr, le compilateur remplace litéralement comme un define, mais en plus intelligent. static const fonctionne aussi, pour arduino (constexpr ne compile pas pour arduino)
    FilterLowPass m_filterSwitch = FilterLowPass(PERIOD_FILTER_SWITCH);
};

#endif

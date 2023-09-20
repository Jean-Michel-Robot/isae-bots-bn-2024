#ifndef __H_SWITCHS
#define __H_SWITCHS
#include "src/Task/Task.h"
#include "src/FilterLowPass/FilterLowPass.h"

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

class SwitchesTask : public Task // heritage public, c'est à dire que tout le monde peut utiliser le fait que Task est parent de SwitchesTask
{
  public :
    SwitchesTask(TaskType type);
    bool isSwitchPressed(bool avant,bool gauche) const;
    bool isSwitchPressed(int index) const;

    static constexpr int INDEX_AVD = 0;
    static constexpr int INDEX_AVG = 1;
    static constexpr int INDEX_ARD = 2;
    static constexpr int INDEX_ARG = 3;

    static constexpr int PIN_RECAL_AVD = 2;
    static constexpr int PIN_RECAL_AVG = 11;
    static constexpr int PIN_RECAL_ARD = 21;
    static constexpr int PIN_RECAL_ARG = 16;
protected :
    void _loop() override; // override car on se superpose a la method Task::_loop() // appele par la methode statique s_executeAllTaskIfNeeded
    SwitchFiltered* m_switches[4];
};
#endif

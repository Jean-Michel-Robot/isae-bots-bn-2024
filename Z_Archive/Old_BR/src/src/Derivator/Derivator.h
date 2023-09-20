#ifndef __DERIVATOR_H_INCLUDED
#define __DERIVATOR_H_INCLUDED
/*
 * classe de dérivation d'une mesure/Valeur
 * calcule la double dérivée
 * filtre les sorties pour des valeurs exploitables
 * basé sur un schéma bloc d'un systeme du second ordre réglé avec omega et Q
 * Les variable d'états du système correspondent à l'entrée filtrée, la dérivée et double dérivée (cf masse ressort avec position vitesse accélération sur une référence mobile)
 *
 * */
class Derivator_2{
public :
    Derivator_2(float omega,float Q, unsigned long t_micros,float satu);
    void update(float input, unsigned long t_micros);
    float getOutput() const;
    float getDerivate() const;
    float getDoubleDerivate() const;
    void RAZ(unsigned long t_micros);
private :
    float m_a1,m_a2,m_a3;
    float m_y,m_yd,m_yd2; // etats internes
    unsigned long m_lastCalc; // dernier moment ou le calcul a été fait
    float m_satu;
};
#endif

# Generátor inverznej kinematiky

Tento program obsahuje zdrojové kódy a súbory k projektu s názvom:

**"Generátor inverznej kinematiky"**

## Obsah
- Generovanie inverznej kinematiky pre 7-DOF robota
- Náhodné generovanie cieľových polôh v rámci pracovného priestoru
- Výpočet inverznej kinematiky pre zadané pozície
- Vizualizácia pohybu robota v prostredí PyBullet

## Požiadavky
Projekt je vytvorený v prostredí Python 3.x a závisí od nasledovných knižníc:
- `pybullet`
- `numpy`

## Spustenie
python GencoKomenska.py

Po spustení sa otvorí PyBullet GUI, v ktorom prebehne simulácia vykonania inverznej kinematiky pre náhodne vygenerované pozície. Robot sa bude pohybovať podľa vypočítaných konfigurácií a následne sa vygeneruje vizualizácia pohybu.

## Prínos
Simulácia je určená na testovanie a vizuálne overovanie inverznej kinematiky pre robot KUKA LBR iiwa.

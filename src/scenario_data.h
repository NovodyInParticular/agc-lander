// Scenario data tables (generated).
#ifndef SCENARIO_DATA_H
#define SCENARIO_DATA_H

typedef struct { int address, value; } ScenarioWord;

extern const ScenarioWord a11_pdi_erasable[];
extern const int a11_pdi_erasable_count;
extern const ScenarioWord a11_pdi_channels[];
extern const int a11_pdi_channel_count;
extern const int a11_pdi_superbank;
extern const double a11_pdi_dscfuel, a11_pdi_ascfuel, a11_pdi_dscemptymass, a11_pdi_ascemptymass;

extern const ScenarioWord a11_liftoff_erasable[];
extern const int a11_liftoff_erasable_count;
extern const ScenarioWord a11_liftoff_channels[];
extern const int a11_liftoff_channel_count;
extern const int a11_liftoff_superbank;
extern const double a11_liftoff_dscfuel, a11_liftoff_ascfuel, a11_liftoff_dscemptymass, a11_liftoff_ascemptymass;

#endif

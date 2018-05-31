within BuildingMpc.Examples.SimulationModels.HeatingSystems;
model Hea900IdealGEO "ideal geothermal system for case 900"
  extends IDEAS.Templates.Interfaces.BaseClasses.HeatingSystem(
  nEmbPorts=1,nTemSen=1);
  IDEAS.Fluid.HeatPumps.Carnot_TCon heaPum(
    redeclare package Medium1 = IDEAS.Media.Water,
    redeclare package Medium2 = IDEAS.Media.Water,
    QCon_flow_nominal=2570,
    dTEva_nominal=-3,
    dTCon_nominal=5,
    use_eta_Carnot_nominal=false,
    COP_nominal=4.5,
    TCon_nominal=308.15,
    TEva_nominal=276.15,
    dp1_nominal=0,
    dp2_nominal=0,
    TAppCon_nominal=5,
    TAppEva_nominal=3,
    QCon_flow_max=2570)
    annotation (Placement(transformation(extent={{-68,4},{-48,24}})));
end Hea900IdealGEO;

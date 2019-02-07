within BuildingMpc.Examples.MPCs;
model MpcCase900GEOTABS_cooling_noBuffTank
  extends UnitTests.MPC.BaseClasses.Mpc(
    final nOut=8,
    final nOpt=5,
    final nSta=181,
    final nMeas=0,
    final nIneq=10,
    final nLLIn=0,
    final nLLOut=0,
    final nLLSta=0,
    final horizonLength=288,
    final numControlIntervals=14,
    final controlTimeStep=900,
    final nModCorCoeff=21,
    final name= "Case900GEOTABS_cooling_noBuffTank");
  Modelica.Blocks.Interfaces.RealOutput Q_cond = getOutput(tableID,3, time)
    annotation (Placement(transformation(extent={{96,50},{116,70}})));
  Modelica.Blocks.Interfaces.RealOutput Tsta = getOutput(tableID,1, time)
    annotation (Placement(transformation(extent={{96,50},{116,70}})));
  Modelica.Blocks.Interfaces.RealOutput W_comp = getOutput(tableID,2, time)
    annotation (Placement(transformation(extent={{96,50},{116,70}})));
  Modelica.Blocks.Interfaces.RealOutput slack = getOutput(tableID,8, time)
    annotation (Placement(transformation(extent={{96,50},{116,70}})));
  Modelica.Blocks.Interfaces.RealOutput u1 = getOutput(tableID,4, time)
    annotation (Placement(transformation(extent={{96,50},{116,70}})));
  Modelica.Blocks.Interfaces.RealOutput u2 = getOutput(tableID,5, time)
    annotation (Placement(transformation(extent={{96,50},{116,70}})));
  Modelica.Blocks.Interfaces.RealOutput u3 = getOutput(tableID,6, time)
    annotation (Placement(transformation(extent={{96,50},{116,70}})));
  Modelica.Blocks.Interfaces.RealOutput u4 = getOutput(tableID,7, time)
    annotation (Placement(transformation(extent={{96,50},{116,70}})));
end MpcCase900GEOTABS_cooling_noBuffTank;

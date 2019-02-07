within BuildingMpc.Examples.MPCs;
model MpcCase900GEOTABS_bufferTank
  extends UnitTests.MPC.BaseClasses.Mpc(
    final nOut=8,
    final nOpt=4,
    final nSta=181,
    final nMeas=0,
    final nIneq=7,
    final nLLIn=0,
    final nLLOut=0,
    final nLLSta=0,
    final horizonLength=24,
    final numControlIntervals=12,
    final controlTimeStep=3600,
    final nModCorCoeff=21,
    final name= "Case900GEOTABS_bufferTank");
  Modelica.Blocks.Interfaces.RealOutput Q_cond = getOutput(tableID,3, time)
    annotation (Placement(transformation(extent={{96,50},{116,70}})));
  Modelica.Blocks.Interfaces.RealOutput Tsta = getOutput(tableID,1, time)
    annotation (Placement(transformation(extent={{96,50},{116,70}})));
  Modelica.Blocks.Interfaces.RealOutput W_comp = getOutput(tableID,2, time)
    annotation (Placement(transformation(extent={{96,50},{116,70}})));
  Modelica.Blocks.Interfaces.RealOutput slack = getOutput(tableID,8, time)
    annotation (Placement(transformation(extent={{96,50},{116,70}})));
  Modelica.Blocks.Interfaces.RealOutput u1 = getOutput(tableID,5, time)
    annotation (Placement(transformation(extent={{96,50},{116,70}})));
  Modelica.Blocks.Interfaces.RealOutput u2 = getOutput(tableID,6, time)
    annotation (Placement(transformation(extent={{96,50},{116,70}})));
  Modelica.Blocks.Interfaces.RealOutput u3 = getOutput(tableID,7, time)
    annotation (Placement(transformation(extent={{96,50},{116,70}})));
  Modelica.Blocks.Interfaces.RealOutput vol_T = getOutput(tableID,4, time)
    annotation (Placement(transformation(extent={{96,50},{116,70}})));
end MpcCase900GEOTABS_bufferTank;

within BuildingMpc.Examples.MPCs.Case900Paper;
model MpcCase900Sou
    extends UnitTests.MPC.BaseClasses.Mpc(
      final nOut=5,
      final nOpt=2,
      final nSta=36,
      final nMeas=0,
      final nIneq=6,
      final nLLIn=0,
      final nLLOut=0,
      final nLLSta=0,
      final horizonLength=24,
      final numControlIntervals=12,
      final controlTimeStep=3600,
      final nModCorCoeff=16,
      final name= "Case900Sou");
    Modelica.Blocks.Interfaces.RealOutput Q_cond = getOutput(tableID,3, time)
      annotation (Placement(transformation(extent={{96,50},{116,70}})));
    Modelica.Blocks.Interfaces.RealOutput Tsta = getOutput(tableID,1, time)
      annotation (Placement(transformation(extent={{96,50},{116,70}})));
    Modelica.Blocks.Interfaces.RealOutput W_comp = getOutput(tableID,2, time)
      annotation (Placement(transformation(extent={{96,50},{116,70}})));
    Modelica.Blocks.Interfaces.RealOutput u1 = getOutput(tableID,4, time)
      annotation (Placement(transformation(extent={{96,50},{116,70}})));
    Modelica.Blocks.Interfaces.RealOutput u2 = getOutput(tableID,5, time)
      annotation (Placement(transformation(extent={{96,50},{116,70}})));
end MpcCase900Sou;

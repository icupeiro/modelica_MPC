within BuildingMpc.Examples.MPCs;
model MpcCase900
  extends UnitTests.MPC.BaseClasses.Mpc(
    final nOut=3,
    final nOpt=2,
    final nSta=30,
    final nMeas=0,
    final controlTimeStep=3600,
    final nModCorCoeff=16,
    final name= "Case900");
  Modelica.Blocks.Interfaces.RealOutput u = getOutput(tableID,1, time)
    annotation (Placement(transformation(extent={{96,50},{116,70}})));
  Modelica.Blocks.Interfaces.RealOutput slack = getOutput(tableID,2, time)
    annotation (Placement(transformation(extent={{96,50},{116,70}})));
  Modelica.Blocks.Interfaces.RealOutput Tsta = getOutput(tableID,3, time)
    annotation (Placement(transformation(extent={{96,50},{116,70}})));
  MpcSignalBus bus
    "Bus connector with control variables and outputs"
    annotation (Placement(transformation(extent={{-20,80},{20,120}})));

  expandable connector MpcSignalBus  "Icon for signal bus"
    extends Modelica.Icons.SignalBus;
  Modelica.Blocks.Interfaces.RealInput u;
  Modelica.Blocks.Interfaces.RealInput slack;
  Modelica.Blocks.Interfaces.RealInput Tsta;
  end MpcSignalBus;
equation
  connect(u, bus.u);
  connect(slack, bus.slack);
  connect(Tsta, bus.Tsta);
end MpcCase900;

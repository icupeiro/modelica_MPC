within BuildingMpc.Examples;
package MPCs
extends Modelica.Icons.ExamplesPackage;

  model MpcCase900
    extends UnitTests.MPC.BaseClasses.Mpc(
      final nOut=1,
      final nOpt=1,
      final nSta=30,
      final nMeas=0,
      final controlTimeStep=3600,
      final nModCorCoeff=16,
      final name= "Case900");
    Modelica.Blocks.Interfaces.RealOutput u = getOutput(tableID,1, time)
      annotation (Placement(transformation(extent={{96,50},{116,70}})));
    MpcSignalBus bus
      "Bus connector with control variables and outputs"
      annotation (Placement(transformation(extent={{-20,80},{20,120}})));

    expandable connector MpcSignalBus  "Icon for signal bus"
      extends Modelica.Icons.SignalBus;
    Modelica.Blocks.Interfaces.RealInput u;
    end MpcSignalBus;
  equation
    connect(u, bus.u);
  end MpcCase900;

  model MpcCase900TABS
    extends UnitTests.MPC.BaseClasses.Mpc(
      final nOut=2,
      final nOpt=1,
      final nSta=30,
      final nMeas=0,
      final controlTimeStep=3600,
      final nModCorCoeff=21,
      final name= "Case900TABS");
    Modelica.Blocks.Interfaces.RealOutput u = getOutput(tableID,1, time)
      annotation (Placement(transformation(extent={{96,50},{116,70}})));
    Modelica.Blocks.Interfaces.RealOutput Tsta = getOutput(tableID,2, time)
      annotation (Placement(transformation(extent={{96,50},{116,70}})));
    MpcSignalBus bus
      "Bus connector with control variables and outputs"
      annotation (Placement(transformation(extent={{-20,80},{20,120}})));

    expandable connector MpcSignalBus  "Icon for signal bus"
      extends Modelica.Icons.SignalBus;
    Modelica.Blocks.Interfaces.RealInput u;
    Modelica.Blocks.Interfaces.RealInput Tsta;
    end MpcSignalBus;
  equation
    connect(u, bus.u);
    connect(Tsta, bus.Tsta);
  end MpcCase900TABS;

  model MpcCase900TABS_2var
    extends UnitTests.MPC.BaseClasses.Mpc(
      final nOut=4,
      final nOpt=3,
      final nSta=30,
      final nMeas=0,
      final controlTimeStep=3600,
      final nModCorCoeff=21,
      final name= "Case900TABS_2var");
    Modelica.Blocks.Interfaces.RealOutput u1 = getOutput(tableID,1, time)
      annotation (Placement(transformation(extent={{96,50},{116,70}})));
    Modelica.Blocks.Interfaces.RealOutput u2 = getOutput(tableID,2, time)
      annotation (Placement(transformation(extent={{96,50},{116,70}})));
    Modelica.Blocks.Interfaces.RealOutput slack = getOutput(tableID,3, time)
      annotation (Placement(transformation(extent={{96,50},{116,70}})));
    Modelica.Blocks.Interfaces.RealOutput Tsta = getOutput(tableID,4, time)
      annotation (Placement(transformation(extent={{96,50},{116,70}})));
    MpcSignalBus bus
      "Bus connector with control variables and outputs"
      annotation (Placement(transformation(extent={{-20,80},{20,120}})));

    expandable connector MpcSignalBus  "Icon for signal bus"
      extends Modelica.Icons.SignalBus;
    Modelica.Blocks.Interfaces.RealInput u1;
    Modelica.Blocks.Interfaces.RealInput u2;
    Modelica.Blocks.Interfaces.RealInput slack;
    Modelica.Blocks.Interfaces.RealInput Tsta;
    end MpcSignalBus;
  equation
    connect(u1, bus.u1);
    connect(u2, bus.u2);
    connect(slack, bus.slack);
    connect(Tsta, bus.Tsta);
  end MpcCase900TABS_2var;

  model MpcCase900TABS_HP
    extends UnitTests.MPC.BaseClasses.Mpc(
      final nOut=5,
      final nOpt=4,
      final nSta=31,
      final nMeas=0,
      final controlTimeStep=3600,
      final nModCorCoeff=21,
      final name= "Case900TABS_HP");
    Modelica.Blocks.Interfaces.RealOutput u1 = getOutput(tableID,1, time)
      annotation (Placement(transformation(extent={{96,50},{116,70}})));
    Modelica.Blocks.Interfaces.RealOutput u2 = getOutput(tableID,2, time)
      annotation (Placement(transformation(extent={{96,50},{116,70}})));
    Modelica.Blocks.Interfaces.RealOutput u3 = getOutput(tableID,3, time)
      annotation (Placement(transformation(extent={{96,50},{116,70}})));
    Modelica.Blocks.Interfaces.RealOutput slack = getOutput(tableID,4, time)
      annotation (Placement(transformation(extent={{96,50},{116,70}})));
    Modelica.Blocks.Interfaces.RealOutput Tsta = getOutput(tableID,5, time)
      annotation (Placement(transformation(extent={{96,50},{116,70}})));
    MpcSignalBus bus
      "Bus connector with control variables and outputs"
      annotation (Placement(transformation(extent={{-20,80},{20,120}})));

    expandable connector MpcSignalBus  "Icon for signal bus"
      extends Modelica.Icons.SignalBus;
    Modelica.Blocks.Interfaces.RealInput u1;
    Modelica.Blocks.Interfaces.RealInput u2;
    Modelica.Blocks.Interfaces.RealInput u3;
    Modelica.Blocks.Interfaces.RealInput slack;
    Modelica.Blocks.Interfaces.RealInput Tsta;
    end MpcSignalBus;
  equation
    connect(u1, bus.u1);
    connect(u2, bus.u2);
    connect(u3, bus.u3);
    connect(slack, bus.slack);
    connect(Tsta, bus.Tsta);
  end MpcCase900TABS_HP;

  model MpcCase900GEOTABS_1bor
    extends UnitTests.MPC.BaseClasses.Mpc(
      final nOut=4,
      final nOpt=3,
      final nSta=151,
      final nMeas=0,
      final controlTimeStep=3600,
      final nModCorCoeff=21,
      final name= "Case900GEOTABS_1bor");
    Modelica.Blocks.Interfaces.RealOutput u1 = getOutput(tableID,1, time)
      annotation (Placement(transformation(extent={{96,50},{116,70}})));
    Modelica.Blocks.Interfaces.RealOutput u2 = getOutput(tableID,2, time)
      annotation (Placement(transformation(extent={{96,50},{116,70}})));
    Modelica.Blocks.Interfaces.RealOutput u3 = getOutput(tableID,3, time)
      annotation (Placement(transformation(extent={{96,50},{116,70}})));
    Modelica.Blocks.Interfaces.RealOutput Tsta = getOutput(tableID,4, time)
      annotation (Placement(transformation(extent={{96,50},{116,70}})));
    MpcSignalBus bus
      "Bus connector with control variables and outputs"
      annotation (Placement(transformation(extent={{-20,80},{20,120}})));

    expandable connector MpcSignalBus  "Icon for signal bus"
      extends Modelica.Icons.SignalBus;
    Modelica.Blocks.Interfaces.RealInput u1;
    Modelica.Blocks.Interfaces.RealInput u2;
    Modelica.Blocks.Interfaces.RealInput u3;
    Modelica.Blocks.Interfaces.RealInput Tsta;
    end MpcSignalBus;
  equation
    connect(u1, bus.u1);
    connect(u2, bus.u2);
    connect(u3, bus.u3);
    connect(Tsta, bus.Tsta);
  end MpcCase900GEOTABS_1bor;
end MPCs;

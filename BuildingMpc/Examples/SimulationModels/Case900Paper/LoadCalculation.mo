within BuildingMpc.Examples.SimulationModels.Case900Paper;
model LoadCalculation
  extends Modelica.Icons.Example;

  parameter Real COP = 4.9;

  Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor temperatureSensor
    annotation (Placement(transformation(extent={{22,-22},{40,-4}})));
  IDEAS.Controls.Continuous.LimPID conPID(
    yMin=0,
    yMax=rectangularZoneTemplate.Q_design - rectangularZoneTemplate.QRH_design,
    controllerType=Modelica.Blocks.Types.SimpleController.PI,
    Ti=10,
    k=5)
    annotation (Placement(transformation(extent={{60,-40},{40,-60}})));

  Modelica.Blocks.Sources.Constant TSetHea(k=21 + 273.15)
    annotation (Placement(transformation(extent={{92,-60},{72,-40}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow prescribedHeat
    annotation (Placement(transformation(extent={{30,-60},{10,-40}})));
  IDEAS.Buildings.Components.RectangularZoneTemplate rectangularZoneTemplate(
    h=2.7,
    redeclare IDEAS.Buildings.Components.ZoneAirModels.WellMixedAir airModel(
        massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState),
    bouTypA=IDEAS.Buildings.Components.Interfaces.BoundaryType.OuterWall,
    bouTypB=IDEAS.Buildings.Components.Interfaces.BoundaryType.OuterWall,
    bouTypC=IDEAS.Buildings.Components.Interfaces.BoundaryType.OuterWall,
    bouTypCei=IDEAS.Buildings.Components.Interfaces.BoundaryType.OuterWall,
    hasWinCei=false,
    redeclare IDEAS.Buildings.Validation.Data.Constructions.LightRoof conTypCei,
    bouTypFlo=IDEAS.Buildings.Components.Interfaces.BoundaryType.BoundaryWall,
    redeclare IDEAS.Buildings.Validation.Data.Constructions.HeavyWall conTypA,
    redeclare IDEAS.Buildings.Validation.Data.Constructions.HeavyWall conTypB,
    redeclare IDEAS.Buildings.Validation.Data.Constructions.HeavyWall conTypC,
    redeclare IDEAS.Buildings.Validation.Data.Constructions.HeavyWall conTypD,
    hasWinA=true,
    fracA=0,
    redeclare IDEAS.Buildings.Components.Shading.Interfaces.ShadingProperties
      shaTypA,
    bouTypD=IDEAS.Buildings.Components.Interfaces.BoundaryType.OuterWall,
    aziA=IDEAS.Types.Azimuth.S,
    mSenFac=0.822,
    l=8,
    w=6,
    n50=0.822*0.5*20,
    redeclare IDEAS.Buildings.Validation.Data.Glazing.GlaBesTest glazingA,
    redeclare package Medium = IDEAS.Media.Air,
    hasWinB=true,
    hasWinC=true,
    hasWinD=true,
    A_winA=3,
    A_winB=3,
    A_winC=3,
    A_winD=3,
    redeclare IDEAS.Buildings.Validation.Data.Constructions.HeavyFloorFH
      conTypFlo,
    redeclare IDEAS.Buildings.Validation.Data.Glazing.GlaBesTest glazingB,
    redeclare IDEAS.Buildings.Validation.Data.Glazing.GlaBesTest glazingC,
    redeclare IDEAS.Buildings.Validation.Data.Glazing.GlaBesTest glazingD,
    hasInt=true,
    lInt=10,
    redeclare IDEAS.Examples.PPD12.Data.InteriorWall10 conTypInt,
    hasEmb=true,
    T_start=296.15,
    fracB=0,
    fracC=0,
    fracD=0)
    annotation (Placement(transformation(extent={{-20,-20},{0,0}})));

  IBPSA.Fluid.Sources.Boundary_pT bou(nPorts=1, redeclare package Medium =
        IDEAS.Media.Air)
    annotation (Placement(transformation(extent={{-46,30},{-26,50}})));
  Modelica.Blocks.Sources.RealExpression HP_load(y=if rectangularZoneTemplate.gainCon.Q_flow
         >= 0.3*conPID.yMax then 0.3*conPID.yMax elseif rectangularZoneTemplate.gainCon.Q_flow
         <= 0.3*conPID1.yMin then 0.3*conPID1.yMin else rectangularZoneTemplate.gainCon.Q_flow)
    annotation (Placement(transformation(extent={{40,30},{60,52}})));
  Modelica.Blocks.Sources.RealExpression groundLoad(y=if HP_load.y > 0 then -
        HP_load.y*(COP - 1)/COP else -HP_load.y)
    annotation (Placement(transformation(extent={{40,8},{60,30}})));
  inner IDEAS.Buildings.Validation.BaseClasses.SimInfoManagerBestest
                                          sim(Tdes=273.15 - 21)
    annotation (Placement(transformation(extent={{-100,80},{-80,100}})));
  IDEAS.Controls.Continuous.LimPID conPID1(
    yMax=0,
    yMin=-conPID.yMax,
    controllerType=Modelica.Blocks.Types.SimpleController.PI,
    Ti=10,
    k=5)
    annotation (Placement(transformation(extent={{60,-74},{40,-94}})));
  Modelica.Blocks.Sources.Constant TSetCoo(k=25 + 273.15)
    annotation (Placement(transformation(extent={{92,-94},{72,-74}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow prescribedCoo
    annotation (Placement(transformation(extent={{32,-94},{12,-74}})));
  Modelica.Blocks.Continuous.Integrator totalHea
    annotation (Placement(transformation(extent={{-40,-60},{-60,-40}})));
  Modelica.Blocks.Continuous.Integrator totalCoo
    annotation (Placement(transformation(extent={{-40,-94},{-60,-74}})));
  Modelica.Blocks.Continuous.Integrator groundBalance
    annotation (Placement(transformation(extent={{72,8},{94,30}})));
equation
  connect(conPID.u_m, temperatureSensor.T)
    annotation (Line(points={{50,-38},{50,-13},{40,-13}}, color={0,0,127}));
  connect(TSetHea.y, conPID.u_s)
    annotation (Line(points={{71,-50},{62,-50}}, color={0,0,127}));
  connect(prescribedHeat.Q_flow, conPID.y)
    annotation (Line(points={{30,-50},{39,-50}}, color={0,0,127}));
  connect(temperatureSensor.port, rectangularZoneTemplate.gainCon) annotation (
      Line(points={{22,-13},{11,-13},{11,-13},{0,-13}}, color={191,0,0}));
  connect(prescribedHeat.port, rectangularZoneTemplate.gainCon) annotation (
      Line(points={{10,-50},{8,-50},{8,-13},{0,-13}}, color={191,0,0}));
  connect(bou.ports[1], rectangularZoneTemplate.port_a)
    annotation (Line(points={{-26,40},{-8,40},{-8,0}}, color={0,127,255}));
  connect(TSetCoo.y, conPID1.u_s)
    annotation (Line(points={{71,-84},{62,-84}}, color={0,0,127}));
  connect(temperatureSensor.T, conPID1.u_m) annotation (Line(points={{40,-13},{46,
          -13},{46,-12},{50,-12},{50,-72}}, color={0,0,127}));
  connect(conPID1.y, prescribedCoo.Q_flow)
    annotation (Line(points={{39,-84},{32,-84}}, color={0,0,127}));
  connect(prescribedCoo.port, rectangularZoneTemplate.gainCon) annotation (Line(
        points={{12,-84},{8,-84},{8,-13},{0,-13}}, color={191,0,0}));
  connect(conPID.y, totalHea.u) annotation (Line(points={{39,-50},{34,-50},{34,
          -60},{-24,-60},{-24,-50},{-38,-50}}, color={0,0,127}));
  connect(conPID1.y, totalCoo.u) annotation (Line(points={{39,-84},{36,-84},{36,
          -96},{-24,-96},{-24,-84},{-38,-84}}, color={0,0,127}));
  connect(groundLoad.y, groundBalance.u)
    annotation (Line(points={{61,19},{69.8,19}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    experiment(
      StopTime=31536000,
      Tolerance=1e-06,
      __Dymola_fixedstepsize=30,
      __Dymola_Algorithm="Euler"),
    __Dymola_experimentSetupOutput,
    __Dymola_experimentFlags(
      Advanced(
        EvaluateAlsoTop=false,
        GenerateVariableDependencies=false,
        OutputModelicaCode=false),
      Evaluate=true,
      OutputCPUtime=true,
      OutputFlatModelica=false));
end LoadCalculation;

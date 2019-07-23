within BuildingMpc.Examples.ControllerModels.Case900Paper;
model Case900SouStaUpd
  extends Modelica.Icons.Example;
  package Glycol = IBPSA.Media.Antifreeze.PropyleneGlycolWater(property_T=278.15, X_a=0.25);
  parameter Real eff = 1.0;

  IBPSA.Fluid.Sources.Boundary_pT bou(nPorts=1, redeclare package Medium =
        BuildingMpc.Media.DryAir)
    annotation (Placement(transformation(extent={{-78,32},{-58,52}})));
  inner IDEAS.Buildings.Validation.BaseClasses.SimInfoManagerBestest
                                          sim(lineariseJModelica=true, Tdes=
        273.15 - 21)
    annotation (Placement(transformation(extent={{-100,80},{-80,100}})));
  Fluid.HeatPumps.HeatPump_y heatPump(
    redeclare package Medium1 = IDEAS.Media.Water,
    dp1_nominal=0,
    dp2_nominal=0,
    redeclare package Medium2 = Glycol,
    Q_nom=(rectangularZoneTemplate.Q_design - rectangularZoneTemplate.QRH_design)
        *0.3,
    Q_con(start=0),
    PLos=0,
    etaCom=1,
    Q_con_max=heatPump.Q_nom*heatPump.y,
    m1_flow_nominal=0.1,
    m2_flow_nominal=0.1,
    COP_expr=5.5) annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=0,
        origin={70,-30})));
protected
  IDEAS.Fluid.Movers.BaseClasses.IdealSource m_flow_sink(
    redeclare final package Medium = IDEAS.Media.Water,
    final allowFlowReversal=false,
    final control_m_flow=true,
    m_flow_small=1e-04,
    m_flow_start=0.05,
    show_T=true)                "Pressure source"
    annotation (Placement(transformation(extent={{40,-90},{20,-70}})));
public
  IBPSA.Fluid.Sources.Boundary_pT source(
    nPorts=2,
    redeclare package Medium = Glycol,
    p=200000,
    use_T_in=true)
              annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-36,70})));
protected
  IDEAS.Fluid.Movers.BaseClasses.IdealSource m_flow_source(
    final allowFlowReversal=false,
    final control_m_flow=true,
    redeclare package Medium = Glycol,
    m_flow_small=1e-04)       "Pressure source"
    annotation (Placement(transformation(extent={{58,62},{78,82}})));
public
  Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow airSystem
    annotation (Placement(transformation(extent={{10,-10},{-10,10}})));
  IDEAS.Fluid.HeatExchangers.RadiantSlab.EmbeddedPipe embeddedPipe(
    redeclare package Medium = IDEAS.Media.Water,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    allowFlowReversal=false,
    A_floor=rectangularZoneTemplate.A,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    dp_nominal=0,
    nParCir=2,
    redeclare IDEAS.Fluid.HeatExchangers.RadiantSlab.BaseClasses.FH_Standard1
      RadSlaCha(
      tabs=false,
      d_a=0.012,
      s_r=0.002,
      nParCir=2,
      S_1=0.03,
      S_2=0.05,
      lambda_b=1.13,
      c_b=1000,
      rho_b=1400),
    T_start(displayUnit="K"),
    computeFlowResistance=false,
    m_flow_nominal=0.1,
    m_flowMin=0.1)
    annotation (Placement(transformation(extent={{-28,-46},{-8,-26}})));
  IBPSA.Fluid.Sources.Boundary_pT sink(
    redeclare package Medium = IDEAS.Media.Water,
    use_T_in=false,
    nPorts=1,
    p=200000) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={-84,-24})));
  Modelica.Blocks.Sources.Constant       mFlowSin(k=0.1)
    annotation (Placement(transformation(extent={{-20,-76},{0,-56}})));
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
    l=8,
    w=6,
    n50=0.822*0.5*20,
    redeclare IDEAS.Buildings.Validation.Data.Glazing.GlaBesTest glazingA,
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
    fracB=0,
    fracC=0,
    fracD=0,
    redeclare package Medium = BuildingMpc.Media.DryAir,
    mSenFac=0.822,
    T_start=296.15)
    annotation (Placement(transformation(extent={{-56,-20},{-36,0}})));

  Modelica.Blocks.Interfaces.RealInput u2(
    min=0,
    start=0,
    max=4274.03,
    nominal=4274.03)
             annotation (Placement(transformation(extent={{-120,-6},{-80,34}})));
  Modelica.Blocks.Interfaces.RealInput u1(
    max=1,
    nominal=1,
    min=0.2,
    start=0.2)
    annotation (Placement(transformation(extent={{-120,-74},{-80,-34}})));
  Modelica.Thermal.HeatTransfer.Components.HeatCapacitor heatCapacitor(C=
        Modelica.Constants.inf, T(fixed=true, start=283.15))
    annotation (Placement(transformation(extent={{-100,56},{-80,76}})));
  Modelica.Blocks.Sources.RealExpression T_source(y=heatCapacitor.T)
    annotation (Placement(transformation(extent={{-76,60},{-56,80}})));
  Modelica.Blocks.Sources.RealExpression nightSetBack(y=if (clock.hour >= 7
         and clock.hour <= 23) then 273.15 + 21 else 273.15 + 16)
    "constraint with night set-back"
    annotation (Placement(transformation(extent={{-4,32},{16,52}})));
  UnitTests.Components.Clock clock
    annotation (Placement(transformation(extent={{-100,26},{-80,46}})));
  Modelica.Blocks.Interfaces.RealInput[4] slack(each min=0)
    annotation (Placement(transformation(extent={{-120,-110},{-80,-70}})));
  Modelica.Blocks.Sources.Constant gasPrice(k=0.061)
    annotation (Placement(transformation(extent={{-46,90},{-36,100}})));
  Modelica.Blocks.Sources.Constant electricityPrice(k=0.204)
    annotation (Placement(transformation(extent={{-30,90},{-20,100}})));
  Modelica.Blocks.Interfaces.RealInput u3(
    min=0,
    max=0.1,
    start=0.1,
    nominal=0.1)
             annotation (Placement(transformation(extent={{-120,22},{-80,62}})));
equation

  connect(source.ports[1], m_flow_source.port_a)
    annotation (Line(points={{-26,72},{58,72}}, color={0,127,255}));
  connect(heatPump.port_a2, m_flow_source.port_b) annotation (Line(points={{80,
          -24},{90,-24},{90,72},{78,72}}, color={0,127,255}));
  connect(heatPump.port_b2, source.ports[2]) annotation (Line(points={{60,-24},
          {52,-24},{52,68},{-26,68}}, color={0,127,255}));
  connect(m_flow_sink.port_b, embeddedPipe.port_a) annotation (Line(points={{20,
          -80},{-54,-80},{-54,-36},{-28,-36}}, color={0,127,255}));
  connect(embeddedPipe.port_b, heatPump.port_a1)
    annotation (Line(points={{-8,-36},{60,-36}}, color={0,127,255}));
  connect(m_flow_sink.port_a, heatPump.port_b1) annotation (Line(points={{40,-80},
          {90,-80},{90,-36},{80,-36}}, color={0,127,255}));
  connect(sink.ports[1], embeddedPipe.port_a) annotation (Line(points={{-84,-34},
          {-84,-36},{-28,-36}}, color={0,127,255}));
  connect(mFlowSin.y, m_flow_sink.m_flow_in)
    annotation (Line(points={{1,-66},{36,-66},{36,-72}}, color={0,0,127}));
  connect(bou.ports[1], rectangularZoneTemplate.port_a)
    annotation (Line(points={{-58,42},{-44,42},{-44,0}}, color={0,127,255}));
  connect(airSystem.port, rectangularZoneTemplate.gainCon) annotation (Line(
        points={{-10,0},{-23,0},{-23,-13},{-36,-13}}, color={191,0,0}));
  for i in 1:embeddedPipe.nDiscr loop
    connect(embeddedPipe.heatPortEmb[i], rectangularZoneTemplate.gainEmb[1])
    annotation (Line(points={{-18,-26},{-28,-26},{-28,-19},{-36,-19}}, color={
          191,0,0}));
  end for;
  connect(airSystem.Q_flow, u2)
    annotation (Line(points={{10,0},{10,14},{-100,14}}, color={0,0,127}));
  connect(T_source.y, source.T_in) annotation (Line(points={{-55,70},{-52,70},{
          -52,74},{-48,74}}, color={0,0,127}));
  connect(u1, heatPump.y) annotation (Line(points={{-100,-54},{-20,-54},{-20,
          -39},{60,-39}}, color={0,0,127}));
  connect(m_flow_source.m_flow_in, u3) annotation (Line(points={{62,80},{62,88},
          {-12,88},{-12,42},{-100,42}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end Case900SouStaUpd;

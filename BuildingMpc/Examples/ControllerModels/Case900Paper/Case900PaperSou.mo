within BuildingMpc.Examples.ControllerModels.Case900Paper;
model Case900PaperSou
  extends Modelica.Icons.Example;
  package Glycol = IBPSA.Media.Antifreeze.PropyleneGlycolWater(property_T=268.15, X_a=0.25);
  parameter Real eff = 0.9;

  IBPSA.Fluid.Sources.Boundary_pT bou(nPorts=1, redeclare package Medium =
        BuildingMpc.Media.DryAir)
    annotation (Placement(transformation(extent={{-78,30},{-58,50}})));
  inner IDEAS.Buildings.Validation.BaseClasses.SimInfoManagerBestest
                                          sim
    annotation (Placement(transformation(extent={{-100,80},{-80,100}})));
  Modelica.Blocks.Sources.RealExpression optVar1(y=0)
    annotation (Placement(transformation(extent={{16,-62},{36,-42}})));
  Fluid.HeatPumps.HeatPump heatPump(
    redeclare package Medium1 = IDEAS.Media.Water,
    m1_flow_nominal=0.3,
    dp1_nominal=0,
    m2_flow_nominal=0.3,
    dp2_nominal=0,
    Q_nom=1500,
    redeclare package Medium2 = Glycol)
                                    annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=0,
        origin={70,-30})));
  Modelica.Blocks.Sources.RealExpression optVar2(y=0)
    annotation (Placement(transformation(extent={{40,-10},{20,10}})));
protected
  IDEAS.Fluid.Movers.BaseClasses.IdealSource m_flow_sink(
    redeclare final package Medium = IDEAS.Media.Water,
    final allowFlowReversal=false,
    final control_m_flow=true,
    final m_flow_small=1e-04) "Pressure source"
    annotation (Placement(transformation(extent={{40,-90},{20,-70}})));
public
  IBPSA.Fluid.Sources.Boundary_pT source(
    use_T_in=false,
    nPorts=2,
    p=200000,
    T=283.15,
    redeclare package Medium = Glycol)
              annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-36,70})));
protected
  IDEAS.Fluid.Movers.BaseClasses.IdealSource m_flow_source(
    final allowFlowReversal=false,
    final control_m_flow=true,
    final m_flow_small=1E-4,
    redeclare package Medium = Glycol)
                             "Pressure source"
    annotation (Placement(transformation(extent={{58,62},{78,82}})));
public
  Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow airSystem
    annotation (Placement(transformation(extent={{10,-10},{-10,10}})));
  IDEAS.Fluid.HeatExchangers.RadiantSlab.EmbeddedPipe embeddedPipe(
    redeclare package Medium = IDEAS.Media.Water,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    redeclare
      IDEAS.Fluid.HeatExchangers.RadiantSlab.BaseClasses.RadiantSlabChar
      RadSlaCha,
    allowFlowReversal=false,
    A_floor=rectangularZoneTemplate.A,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    dp_nominal=0,
    m_flow_nominal=0.05)
    annotation (Placement(transformation(extent={{-28,-46},{-8,-26}})));
  IBPSA.Fluid.Sources.Boundary_pT sink(
    redeclare package Medium = IDEAS.Media.Water,
    use_T_in=false,
    nPorts=1,
    p=200000) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={-84,-24})));
  Modelica.Blocks.Sources.RealExpression mFlowSou(y=0.05)
    annotation (Placement(transformation(extent={{20,78},{40,98}})));
  Modelica.Blocks.Sources.RealExpression mFlowSin(y=0.05)
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
    mSenFac=0.822,
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
    T_start=296.15)
    annotation (Placement(transformation(extent={{-56,-20},{-36,0}})));
equation
  connect(optVar1.y, heatPump.Tcon_out)
    annotation (Line(points={{37,-52},{42,-52},{42,-40},{60,-40},{60,-39}},
                                                          color={0,0,127}));
  connect(source.ports[1], m_flow_source.port_a)
    annotation (Line(points={{-26,72},{58,72}}, color={0,127,255}));
  connect(heatPump.port_a2, m_flow_source.port_b) annotation (Line(points={{80,
          -24},{90,-24},{90,72},{78,72}}, color={0,127,255}));
  connect(heatPump.port_b2, source.ports[2]) annotation (Line(points={{60,-24},
          {52,-24},{52,68},{-26,68}}, color={0,127,255}));
  connect(optVar2.y, airSystem.Q_flow)
    annotation (Line(points={{19,0},{10,0}}, color={0,0,127}));
  connect(m_flow_sink.port_b, embeddedPipe.port_a) annotation (Line(points={{20,
          -80},{-54,-80},{-54,-36},{-28,-36}}, color={0,127,255}));
  connect(embeddedPipe.port_b, heatPump.port_a1)
    annotation (Line(points={{-8,-36},{60,-36}}, color={0,127,255}));
  connect(m_flow_sink.port_a, heatPump.port_b1) annotation (Line(points={{40,
          -80},{90,-80},{90,-36},{80,-36}}, color={0,127,255}));
  connect(sink.ports[1], embeddedPipe.port_a) annotation (Line(points={{-84,-34},
          {-84,-36},{-28,-36}}, color={0,127,255}));
  connect(mFlowSou.y, m_flow_source.m_flow_in)
    annotation (Line(points={{41,88},{62,88},{62,80}}, color={0,0,127}));
  connect(mFlowSin.y, m_flow_sink.m_flow_in)
    annotation (Line(points={{1,-66},{36,-66},{36,-72}}, color={0,0,127}));
  connect(bou.ports[1], rectangularZoneTemplate.port_a)
    annotation (Line(points={{-58,40},{-44,40},{-44,0}}, color={0,127,255}));
  connect(airSystem.port, rectangularZoneTemplate.gainCon) annotation (Line(
        points={{-10,0},{-23,0},{-23,-13},{-36,-13}}, color={191,0,0}));
  connect(embeddedPipe.heatPortEmb, rectangularZoneTemplate.gainEmb)
    annotation (Line(points={{-18,-26},{-28,-26},{-28,-19},{-36,-19}}, color={
          191,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end Case900PaperSou;

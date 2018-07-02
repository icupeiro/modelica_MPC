within BuildingMpc.Examples.ControllerModels;
model Case900GEOTABS
"Controller model for the BESTEST Case900 with TABS and heat pump with a single borehole model; the optimization variables are the outlet temperature of the HP and the mass flows through HP cond/evap"
  extends Modelica.Icons.Example;
  IDEAS.Buildings.Components.RectangularZoneTemplate rectangularZoneTemplate(
    h=2.7,
    redeclare package Medium = BuildingMpc.Media.DryAir,
    redeclare IDEAS.Buildings.Components.ZoneAirModels.WellMixedAir
                                                    airModel(massDynamics=
          Modelica.Fluid.Types.Dynamics.SteadyState),
    bouTypA=IDEAS.Buildings.Components.Interfaces.BoundaryType.OuterWall,
    bouTypB=IDEAS.Buildings.Components.Interfaces.BoundaryType.OuterWall,
    bouTypC=IDEAS.Buildings.Components.Interfaces.BoundaryType.OuterWall,
    bouTypCei=IDEAS.Buildings.Components.Interfaces.BoundaryType.OuterWall,
    hasWinCei=false,
    redeclare IDEAS.Buildings.Validation.Data.Constructions.LightRoof conTypCei,
    redeclare IDEAS.Buildings.Validation.Data.Constructions.HeavyFloor
                                            conTypFlo,
    redeclare IDEAS.Buildings.Validation.Data.Constructions.HeavyWall
                                           conTypA,
    redeclare IDEAS.Buildings.Validation.Data.Constructions.HeavyWall
                                           conTypB,
    redeclare IDEAS.Buildings.Validation.Data.Constructions.HeavyWall
                                           conTypC,
    redeclare IDEAS.Buildings.Validation.Data.Constructions.HeavyWall
                                           conTypD,
    hasWinA=true,
    fracA=0,
    redeclare IDEAS.Buildings.Components.Shading.Interfaces.ShadingProperties
      shaTypA,
    hasWinB=false,
    hasWinC=false,
    hasWinD=false,
    bouTypD=IDEAS.Buildings.Components.Interfaces.BoundaryType.OuterWall,
    aziA=IDEAS.Types.Azimuth.S,
    mSenFac=0.822,
    l=8,
    w=6,
    A_winA=12,
    n50=0.822*0.5*20,
    redeclare IDEAS.Buildings.Validation.Data.Glazing.GlaBesTest
      glazingA,
    bouTypFlo=IDEAS.Buildings.Components.Interfaces.BoundaryType.External,
    useFluPor=true,
    T_start=293.15)
    annotation (Placement(transformation(extent={{-20,-20},{0,0}})));
  inner IDEAS.BoundaryConditions.SimInfoManager sim(lineariseJModelica=true)
    "Simulation information manager for climate data"
    annotation (Placement(transformation(extent={{-100,60},{-80,80}})));
  IBPSA.Fluid.Sources.Boundary_pT bou(nPorts=1, redeclare package Medium =
        BuildingMpc.Media.DryAir)
    annotation (Placement(transformation(extent={{-46,30},{-26,50}})));
  Modelica.Blocks.Sources.RealExpression optVar1
    annotation (Placement(transformation(extent={{-86,-20},{-66,0}})));
  IDEAS.Buildings.Components.BoundaryWall boundaryWall(
    redeclare IDEAS.Buildings.Validation.Data.Constructions.HeavyFloor
      constructionType,
    inc=IDEAS.Types.Tilt.Floor,
    azi=rectangularZoneTemplate.aziA,
    A=rectangularZoneTemplate.A) annotation (Placement(transformation(
        extent={{-6,-10},{6,10}},
        rotation=90,
        origin={-8,-38})));
  IDEAS.Fluid.HeatExchangers.RadiantSlab.EmbeddedPipe embeddedPipe(
    redeclare package Medium = IDEAS.Media.Water,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    redeclare
      IDEAS.Fluid.HeatExchangers.RadiantSlab.BaseClasses.RadiantSlabChar
      RadSlaCha,
    allowFlowReversal=false,
    A_floor=rectangularZoneTemplate.A,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    m_flow_nominal=0.2,
    dp_nominal=0)
    annotation (Placement(transformation(extent={{16,-70},{36,-50}})));
  IBPSA.Fluid.Sources.Boundary_pT source(
    redeclare package Medium = IDEAS.Media.Water,
    use_T_in=false,
    p=200000,
    nPorts=1)      annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-36,70})));
  Modelica.Blocks.Sources.RealExpression optVar2
    annotation (Placement(transformation(extent={{-86,-34},{-66,-14}})));
protected
  IDEAS.Fluid.Movers.BaseClasses.IdealSource m_flow_sink(
    redeclare final package Medium = IDEAS.Media.Water,
    final allowFlowReversal=false,
    final control_m_flow=true,
    final m_flow_small=1e-04) "Pressure source"
    annotation (Placement(transformation(extent={{-44,-70},{-24,-50}})));
protected
  IDEAS.Fluid.Movers.BaseClasses.IdealSource m_flow_source(
    redeclare final package Medium = IDEAS.Media.Water,
    final allowFlowReversal=false,
    final control_m_flow=true,
    final m_flow_small=1e-4) "Pressure source"
    annotation (Placement(transformation(extent={{40,62},{60,82}})));
public
  Modelica.Blocks.Sources.RealExpression optVar3
    annotation (Placement(transformation(extent={{-72,80},{-52,100}})));
  IBPSA.Fluid.Sources.Boundary_pT sink(
    redeclare package Medium = IDEAS.Media.Water,
    use_T_in=false,
    nPorts=1,
    p=200000) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={-78,-50})));
  Fluid.HeatPumps.HeatPump heatPump(
    redeclare package Medium1 = IDEAS.Media.Water,
    redeclare package Medium2 = IDEAS.Media.Water,
    dp2_nominal=0,
    dp1_nominal=0,
    m1_flow_nominal=0.2,
  m2_flow_nominal=0.5)
    annotation (Placement(transformation(extent={{60,-44},{80,-64}})));
  IBPSA.Fluid.HeatExchangers.GroundHeatExchangers.Data.BorefieldData.ExampleBorefieldData
    borFieDat(filDat=
        IBPSA.Fluid.HeatExchangers.GroundHeatExchangers.Data.FillingData.Bentonite(
        steadyState=true), soiDat=
        IBPSA.Fluid.HeatExchangers.GroundHeatExchangers.Data.SoilData.SandStone(
        steadyState=true))
    annotation (Placement(transformation(extent={{-100,80},{-80,100}})));
  Fluid.HeatExchangers.GroundHeatExchangers.Development.MultipleBorehole
                                                           multipleBorehole(
    redeclare package Medium = IDEAS.Media.Water,
    soilTemp=273.15 + 10.8,
    borFieDat=borFieDat,
    dp_nominal=0,
  m_flow_nominal=0.5)
    annotation (Placement(transformation(extent={{52,0},{32,20}})));
equation
  connect(bou.ports[1], rectangularZoneTemplate.port_a)
    annotation (Line(points={{-26,40},{-8,40},{-8,0}}, color={0,127,255}));
  connect(rectangularZoneTemplate.proBusFlo, boundaryWall.propsBus_a)
    annotation (Line(
      points={{-10,-16},{-10,-33}},
      color={255,204,51},
      thickness=0.5));
  connect(embeddedPipe.heatPortEmb, boundaryWall.port_emb) annotation (Line(
        points={{26,-50},{26,-50},{26,-38},{2,-38}}, color={191,0,0}));
  connect(m_flow_sink.port_b, embeddedPipe.port_a)
    annotation (Line(points={{-24,-60},{16,-60}}, color={0,127,255}));
  connect(optVar1.y, m_flow_sink.m_flow_in) annotation (Line(points={{-65,-10},
          {-40,-10},{-40,-52},{-40,-52}}, color={0,0,127}));
  connect(optVar3.y, m_flow_source.m_flow_in) annotation (Line(points={{-51,90},
          {-22,90},{44,90},{44,80}},        color={0,0,127}));
  connect(optVar2.y, heatPump.Tcon_out) annotation (Line(points={{-65,-24},{-60,
          -24},{-60,-80},{48,-80},{48,-63},{60,-63}}, color={0,0,127}));
  connect(embeddedPipe.port_b, heatPump.port_a1)
    annotation (Line(points={{36,-60},{60,-60}}, color={0,127,255}));
  connect(m_flow_sink.port_a, heatPump.port_b1) annotation (Line(points={{-44,-60},
          {-70,-60},{-70,-90},{92,-90},{92,-60},{80,-60}}, color={0,127,255}));
  connect(sink.ports[1], m_flow_sink.port_a) annotation (Line(points={{-78,-60},
          {-61,-60},{-44,-60}}, color={0,127,255}));
  connect(m_flow_source.port_b, heatPump.port_a2) annotation (Line(points={{60,72},
          {60,72},{88,72},{88,-48},{80,-48}}, color={0,127,255}));
  connect(source.ports[1], m_flow_source.port_a) annotation (Line(points={{-26,70},
          {-26,72},{-26,72},{40,72}}, color={0,127,255}));
connect(heatPump.port_b2, multipleBorehole.port_a) annotation (Line(
      points={{60,-48},{60,-48},{60,10},{52,10}}, color={0,127,255}));
connect(multipleBorehole.port_b, m_flow_source.port_a) annotation (Line(
      points={{32,10},{20,10},{20,72},{40,72}}, color={0,127,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    experiment(
      StopTime=31536000,
      __Dymola_NumberOfIntervals=15000,
      Tolerance=1e-06,
      __Dymola_fixedstepsize=10,
      __Dymola_Algorithm="Euler"),
    Documentation(info="<html>
<p>
Template for setting up MPC problem using BESTEST case 900 example model. 
Realexpression <code>optVar</code> can be used as a template for
an optimisation variable.
</p>
</html>", revisions="<html>
<ul>
<li>
November 14, 2016 by Filip Jorissen:<br/>
First implementation
</li>
</ul>
</html>"),
    __Dymola_Commands(file=
          "Resources/Scripts/Dymola/Buildings/Validation/Tests/ZoneTemplateVerification.mos"
        "Simulate and plot"),
    __Dymola_experimentSetupOutput,
    __Dymola_experimentFlags(
      Advanced(
        GenerateVariableDependencies=false,
        OutputModelicaCode=false,
        InlineMethod=0,
        InlineOrder=2,
        InlineFixedStep=0.001),
      Evaluate=false,
      OutputCPUtime=false,
      OutputFlatModelica=false));
end Case900GEOTABS;

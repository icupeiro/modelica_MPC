within BuildingMpc.Examples.ControllerModels;
model Case900TABS
  "Controller model for the BESTEST Case900 with TABS; the optimization variable is the heat flux through TABS"
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
    T_start=293.15,
    bouTypFlo=IDEAS.Buildings.Components.Interfaces.BoundaryType.External)
    annotation (Placement(transformation(extent={{-20,-20},{0,0}})));
  inner IDEAS.BoundaryConditions.SimInfoManager sim(lineariseJModelica=true)
    "Simulation information manager for climate data"
    annotation (Placement(transformation(extent={{-100,60},{-80,80}})));
  IBPSA.Fluid.Sources.Boundary_pT bou(nPorts=1, redeclare package Medium =
        BuildingMpc.Media.DryAir)
    annotation (Placement(transformation(extent={{-46,30},{-26,50}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow preHeaFlo(final
      alpha=0) annotation (Placement(transformation(extent={{40,-48},{20,-28}})));
  Modelica.Blocks.Sources.RealExpression optVar
    annotation (Placement(transformation(extent={{80,-48},{60,-28}})));
  IDEAS.Buildings.Components.BoundaryWall boundaryWall(
    redeclare IDEAS.Buildings.Validation.Data.Constructions.HeavyFloor
      constructionType,
    inc=IDEAS.Types.Tilt.Floor,
    azi=rectangularZoneTemplate.aziA,
    A=rectangularZoneTemplate.A) annotation (Placement(transformation(
        extent={{-6,-10},{6,10}},
        rotation=90,
        origin={-8,-38})));
equation
  connect(bou.ports[1], rectangularZoneTemplate.port_a)
    annotation (Line(points={{-26,40},{-8,40},{-8,0}}, color={0,127,255}));
  connect(optVar.y,preHeaFlo. Q_flow)
    annotation (Line(points={{59,-38},{40,-38}}, color={0,0,127}));
  connect(rectangularZoneTemplate.proBusFlo, boundaryWall.propsBus_a)
    annotation (Line(
      points={{-10,-16},{-10,-33}},
      color={255,204,51},
      thickness=0.5));
  connect(preHeaFlo.port, boundaryWall.port_emb[1])
    annotation (Line(points={{20,-38},{2,-38}}, color={191,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    experiment(
      StartTime=1420000000,
      StopTime=1430000000,
      __Dymola_fixedstepsize=1,
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
        "Simulate and plot"));
end Case900TABS;

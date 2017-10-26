within BuildingMpc;
model Case900
  "MPC template based on bestest case 900"
  extends Modelica.Icons.Example;
  IDEAS.Buildings.Components.RectangularZoneTemplate rectangularZoneTemplate(
    h=2.7,
    redeclare package Medium = Solarwind.Media.DryAir,
    redeclare IDEAS.Buildings.Components.ZoneAirModels.WellMixedAir
                                                    airModel(massDynamics=
          Modelica.Fluid.Types.Dynamics.SteadyState),
    T_start=293.15,
    bouTypA=IDEAS.Buildings.Components.Interfaces.BoundaryType.OuterWall,
    bouTypB=IDEAS.Buildings.Components.Interfaces.BoundaryType.OuterWall,
    bouTypC=IDEAS.Buildings.Components.Interfaces.BoundaryType.OuterWall,
    bouTypCei=IDEAS.Buildings.Components.Interfaces.BoundaryType.OuterWall,
    hasWinCei=false,
    redeclare IDEAS.Buildings.Validation.Data.Constructions.LightRoof conTypCei,
    bouTypFlo=IDEAS.Buildings.Components.Interfaces.BoundaryType.BoundaryWall,
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
      glazingA)
    annotation (Placement(transformation(extent={{-20,-20},{0,0}})));
  inner IDEAS.BoundaryConditions.SimInfoManager sim(lineariseJModelica=true)
    "Simulation information manager for climate data"
    annotation (Placement(transformation(extent={{-100,60},{-80,80}})));
  IBPSA.Fluid.Sources.Boundary_pT bou(nPorts=1, redeclare package Medium =
        Solarwind.Media.DryAir)
    annotation (Placement(transformation(extent={{-46,30},{-26,50}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow preHeaFlo(final
      alpha=0) annotation (Placement(transformation(extent={{40,-22},{20,-2}})));
  Modelica.Blocks.Sources.RealExpression optVar
    annotation (Placement(transformation(extent={{80,-22},{60,-2}})));
equation
  connect(bou.ports[1], rectangularZoneTemplate.port_a)
    annotation (Line(points={{-26,40},{-8,40},{-8,0}}, color={0,127,255}));
  connect(optVar.y,preHeaFlo. Q_flow)
    annotation (Line(points={{59,-12},{40,-12}}, color={0,0,127}));
  connect(preHeaFlo.port, rectangularZoneTemplate.gainRad) annotation (Line(
        points={{20,-12},{10,-12},{10,-16},{0,-16}}, color={191,0,0}));
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
end Case900;

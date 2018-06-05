within BuildingMpc.Examples.Optimizations;
model Case900GEOTABS_1bor
  "Controller model for the BESTEST Case900 with TABS and heat pump with a single borehole model; the optimization variables are the outlet temperature of the HP and the mass flows through HP cond/evap"
  extends BuildingMpc.Examples.ControllerModels.Case900GEOTABS_1bor(
    optVar3(y=mpcCase900GEOTABS_1bor.yOpt[3]),
    optVar1(y=mpcCase900GEOTABS_1bor.yOpt[1]),
    optVar2(y=mpcCase900GEOTABS_1bor.yOpt[2]),
    singleBorehole(dp_nominal=0));
  MPCs.MpcCase900GEOTABS_1bor mpcCase900GEOTABS_1bor
    annotation (Placement(transformation(extent={{-94,20},{-74,40}})));
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
end Case900GEOTABS_1bor;

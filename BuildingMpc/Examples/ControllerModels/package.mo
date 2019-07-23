within BuildingMpc.Examples;
package ControllerModels
  extends Modelica.Icons.ExamplesPackage;





  model ClosedLoopCase900GEOTABS_cooling
    extends Case900GEOTABS_cooling(
      optVar1 = mpc.optVar1,
      optVar2 = mpc.optVar2,
      optVar3 = mpc.optVar3,
      slack = mpc.slack);
    MpcCase900GEOTABS_cooling mpc
      annotation (Placement(transformation(extent={{-100,80},{-80,100}})));

    model MpcCase900GEOTABS_cooling
      extends UnitTests.MPC.BaseClasses.Mpc(
        final nOut=8,
        final nOpt=4,
        final nSta=31,
        final nMeas=0,
        final nIneq=7,
        final nLLIn=0,
        final nLLOut=0,
        final nLLSta=0,
        final horizonLength=76,
        final numControlIntervals=14,
        final controlTimeStep=3600,
        final nModCorCoeff=21,
        final name= "Case900GEOTABS_cooling");
      Modelica.Blocks.Interfaces.RealOutput Q_cond = getOutput(tableID,3, time)
        annotation (Placement(transformation(extent={{96,50},{116,70}})));
      Modelica.Blocks.Interfaces.RealOutput Tsta = getOutput(tableID,1, time)
        annotation (Placement(transformation(extent={{96,50},{116,70}})));
      Modelica.Blocks.Interfaces.RealOutput W_comp = getOutput(tableID,2, time)
        annotation (Placement(transformation(extent={{96,50},{116,70}})));
      Modelica.Blocks.Interfaces.RealOutput optVar1 = getOutput(tableID,5, time)
        annotation (Placement(transformation(extent={{96,50},{116,70}})));
      Modelica.Blocks.Interfaces.RealOutput optVar2 = getOutput(tableID,6, time)
        annotation (Placement(transformation(extent={{96,50},{116,70}})));
      Modelica.Blocks.Interfaces.RealOutput optVar3 = getOutput(tableID,7, time)
        annotation (Placement(transformation(extent={{96,50},{116,70}})));
      Modelica.Blocks.Interfaces.RealOutput slack = getOutput(tableID,8, time)
        annotation (Placement(transformation(extent={{96,50},{116,70}})));
      Modelica.Blocks.Interfaces.RealOutput vol_T = getOutput(tableID,4, time)
        annotation (Placement(transformation(extent={{96,50},{116,70}})));
    end MpcCase900GEOTABS_cooling;
  end ClosedLoopCase900GEOTABS_cooling;
end ControllerModels;

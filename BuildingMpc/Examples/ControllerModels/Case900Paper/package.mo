within BuildingMpc.Examples.ControllerModels;
package Case900Paper
  model ClosedLoopCase900Cooling
    extends Case900Cooling(
      slack = mpc.slack,
      u1 = mpc.u1,
      u2 = mpc.u2,
      u3 = mpc.u3,
      u4 = mpc.u4,
      u5 = mpc.u5);
    MpcCase900Cooling mpc
      annotation (Placement(transformation(extent={{-100,80},{-80,100}})));

    model MpcCase900Cooling
      extends UnitTests.MPC.BaseClasses.Mpc(
        final nOut=19,
        final nOpt=8,
        final nSta=188,
        final nMeas=0,
        final nIneq=7,
        final nLLIn=0,
        final nLLOut=0,
        final nLLSta=0,
        final horizonLength=168,
        final numControlIntervals=13,
        final controlTimeStep=3600,
        final nModCorCoeff=33,
        final name= "Case900Cooling");
      Modelica.Blocks.Interfaces.RealOutput COP = getOutput(tableID,6, time)
        annotation (Placement(transformation(extent={{96,50},{116,70}})));
      Modelica.Blocks.Interfaces.RealOutput Q_con_max = getOutput(tableID,7, time)
        annotation (Placement(transformation(extent={{96,50},{116,70}})));
      Modelica.Blocks.Interfaces.RealOutput Q_cond = getOutput(tableID,3, time)
        annotation (Placement(transformation(extent={{96,50},{116,70}})));
      Modelica.Blocks.Interfaces.RealOutput TAir = getOutput(tableID,1, time)
        annotation (Placement(transformation(extent={{96,50},{116,70}})));
      Modelica.Blocks.Interfaces.RealOutput TRad = getOutput(tableID,4, time)
        annotation (Placement(transformation(extent={{96,50},{116,70}})));
      Modelica.Blocks.Interfaces.RealOutput TRet = getOutput(tableID,5, time)
        annotation (Placement(transformation(extent={{96,50},{116,70}})));
      Modelica.Blocks.Interfaces.RealOutput T_con_in = getOutput(tableID,8, time)
        annotation (Placement(transformation(extent={{96,50},{116,70}})));
      Modelica.Blocks.Interfaces.RealOutput T_con_out = getOutput(tableID,9, time)
        annotation (Placement(transformation(extent={{96,50},{116,70}})));
      Modelica.Blocks.Interfaces.RealOutput T_eva_in = getOutput(tableID,10, time)
        annotation (Placement(transformation(extent={{96,50},{116,70}})));
      Modelica.Blocks.Interfaces.RealOutput T_eva_out = getOutput(tableID,11, time)
        annotation (Placement(transformation(extent={{96,50},{116,70}})));
      Modelica.Blocks.Interfaces.RealOutput W_comp = getOutput(tableID,2, time)
        annotation (Placement(transformation(extent={{96,50},{116,70}})));
      Modelica.Blocks.Interfaces.RealOutput slack[3] = {
        getOutput(tableID,14, time),
        getOutput(tableID,15, time),
        getOutput(tableID,16, time)}
        annotation (Placement(transformation(extent={{96,50},{116,70}})));
      Modelica.Blocks.Interfaces.RealOutput u1 = getOutput(tableID,13, time)
        annotation (Placement(transformation(extent={{96,50},{116,70}})));
      Modelica.Blocks.Interfaces.RealOutput u2 = getOutput(tableID,12, time)
        annotation (Placement(transformation(extent={{96,50},{116,70}})));
      Modelica.Blocks.Interfaces.RealOutput u3 = getOutput(tableID,17, time)
        annotation (Placement(transformation(extent={{96,50},{116,70}})));
      Modelica.Blocks.Interfaces.RealOutput u4 = getOutput(tableID,18, time)
        annotation (Placement(transformation(extent={{96,50},{116,70}})));
      Modelica.Blocks.Interfaces.RealOutput u5 = getOutput(tableID,19, time)
        annotation (Placement(transformation(extent={{96,50},{116,70}})));
    end MpcCase900Cooling;
  end ClosedLoopCase900Cooling;
end Case900Paper;

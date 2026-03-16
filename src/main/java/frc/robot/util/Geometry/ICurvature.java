package frc.robot.util.Geometry;

public interface ICurvature<S> extends State<S> {
  double getCurvature();

  double getDCurvatureDs();
}

package frc.robot.util.Geometry;

import frc.robot.util.Util.CSVWritable;
import frc.robot.util.Util.Interpolable;

public interface State<S> extends Interpolable<S>, CSVWritable {
  double distance(final S other);

  S add(S other);

  boolean equals(final Object other);

  String toString();

  String toCSV();
}

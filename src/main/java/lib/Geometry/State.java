package lib.Geometry;

import lib.Util.CSVWritable;
import lib.Util.Interpolable;

public interface State<S> extends Interpolable<S>, CSVWritable {
  double distance(final S other);

  S add(S other);

  boolean equals(final Object other);

  String toString();

  String toCSV();
}

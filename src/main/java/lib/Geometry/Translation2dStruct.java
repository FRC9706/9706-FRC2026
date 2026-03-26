// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package lib.Geometry;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.struct.Struct;
import java.nio.ByteBuffer;

public class Translation2dStruct implements Struct<lib.Geometry.Translation2d> {
  @Override
  public Class<lib.Geometry.Translation2d> getTypeClass() {
    return lib.Geometry.Translation2d.class;
  }

  @Override
  public String getTypeName() {
    return "Translation2d+";
  }

  @Override
  public int getSize() {
    return Translation2d.struct.getSize();
  }

  @Override
  public String getSchema() {
    return "Translation2d translation";
  }

  @Override
  public lib.Geometry.Translation2d unpack(ByteBuffer bb) {
    return new lib.Geometry.Translation2d(Translation2d.struct.unpack(bb));
  }

  @Override
  public void pack(ByteBuffer bb, lib.Geometry.Translation2d value) {
    Translation2d.struct.pack(bb, value.wpi());
  }

  @Override
  public boolean isImmutable() {
    return true;
  }
}

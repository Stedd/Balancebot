void motorControl()
{
  //Backward
  if (mapValue[2] < 0)
  {
    if (mapValue[3] > 0)
    {
      analogWrite(IN3, -mapValue[2] * ((255 - mapValue[3]) / 255.0));
      analogWrite(IN1, -mapValue[2]);
    }
    if (mapValue[3] < 0)
    {
      analogWrite(IN3, -mapValue[2]);
      analogWrite(IN1, -mapValue[2] * ((255 + mapValue[3]) / 255.0));
    }
    analogWrite(IN2, LOW);
    analogWrite(IN4, LOW);
  }

  //Forward
  if (mapValue[2] > 0)
  {
    if (mapValue[3] > 0)
    {
      analogWrite(IN4, mapValue[2] * ((255 - mapValue[3]) / 255.0));
      analogWrite(IN2, mapValue[2]);
    }
    if (mapValue[3] < 0)
    {
      analogWrite(IN4, mapValue[2]);
      analogWrite(IN2, mapValue[2] * ((255 + mapValue[3]) / 255.0));
    }
    analogWrite(IN1, LOW);
    analogWrite(IN3, LOW);
  }

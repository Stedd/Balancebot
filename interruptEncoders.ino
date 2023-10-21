//Interrupt routines
void IRAM_ATTR m1_A_changed() {
  M1_A_state = digitalRead(M1_ENC_A);
  M1_B_state = digitalRead(M1_ENC_B);

  //Rising
  if (M1_A_state == HIGH) {
    if (M1_B_state == HIGH) {
      m1Raw = m1Raw - 1;
    } else if (M1_B_state == LOW) {
      m1Raw = m1Raw + 1;
    }
  }

  //Falling
  else if (M1_A_state == LOW) {
    if (M1_B_state == HIGH) {
      m1Raw = m1Raw + 1;
    } else if (M1_B_state == LOW) {
      m1Raw = m1Raw - 1;
    }
  }
}


void IRAM_ATTR m1_B_changed() {
  M1_A_state = digitalRead(M1_ENC_A);
  M1_B_state = digitalRead(M1_ENC_B);

  //Rising
  if (M1_B_state == HIGH) {
    if (M1_A_state == HIGH) {
      m1Raw = m1Raw + 1;
    } else if (M1_A_state == LOW) {
      m1Raw = m1Raw - 1;
    }
  }

  //Falling
  else if (M1_B_state == LOW) {
    if (M1_A_state == HIGH) {
      m1Raw = m1Raw - 1;
    } else if (M1_A_state == LOW) {
      m1Raw = m1Raw + 1;
    }
  }
}

void IRAM_ATTR m2_A_changed() {
  M2_A_state = digitalRead(M2_ENC_A);
  M2_B_state = digitalRead(M2_ENC_B);

  //Rising
  if (M2_A_state == HIGH) {
    if (M2_B_state == HIGH) {
      m2Raw = m2Raw + 1;
    } else if (M2_B_state == LOW) {
      m2Raw = m2Raw - 1;
    }
  }

  //Falling
  else if (M2_A_state == LOW) {
    if (M2_B_state == HIGH) {
      m2Raw = m2Raw - 1;
    } else if (M2_B_state == LOW) {
      m2Raw = m2Raw + 1;
    }
  }
}


void IRAM_ATTR m2_B_changed() {
  M2_A_state = digitalRead(M2_ENC_A);
  M2_B_state = digitalRead(M2_ENC_B);

  //Rising
  if (M2_B_state == HIGH) {
    if (M2_A_state == HIGH) {
      m2Raw = m2Raw - 1;
    } else if (M2_A_state == LOW) {
      m2Raw = m2Raw + 1;
    }
  }

  //Falling
  else if (M2_B_state == LOW) {
    if (M2_A_state == HIGH) {
      m2Raw = m2Raw + 1;
    } else if (M2_A_state == LOW) {
      m2Raw = m2Raw - 1;
    }
  }
}


void initEncoderInterrupt() {
  pinMode(M1_ENC_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(M1_ENC_A), m1_A_changed, CHANGE);

  pinMode(M1_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(M1_ENC_B), m1_B_changed, CHANGE);

  pinMode(M2_ENC_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(M2_ENC_A), m2_A_changed, CHANGE);

  pinMode(M2_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(M2_ENC_B), m2_B_changed, CHANGE);
}

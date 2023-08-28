# gantry


## Rest API docs

---

#### **Set Session ID**

- **Endpoint URL**: `/`
- **Method**: POST
- **Headers**: `Content-Type: application/json`
- **Request Body**:

  ```json
  {
    "session_id": "string"
  }
  ```

- **Description**: Sets the session_id for the server.
- **Response**:
  - **Status Codes**:
    - `200`: Session ID received
    - `400`: No session_id provided in JSON body
    - `500`: Failed to parse JSON
- **Errors**:
  - `Content-Type must be application/json`
  - `Failed to parse JSON`
  - `No session_id provided in JSON body`

---

#### **Get Session ID**

- **Endpoint URL**: `/`
- **Method**: GET
- **Headers**: `session_id: string`
- **Description**: Retrieves and verifies the session_id.
- **Response**:
  - **Status Codes**:
    - `200`: Session ID verified
    - `400`: No session_id provided
    - `403`: Forbidden: Incorrect session ID

---

#### **Set Target Waypoint**

- **Endpoint URL**: `/target_waypoint`
- **Method**: POST
- **Headers**: `Content-Type: application/json`
- **Request Body**:

  ```json
  {
    "value": int
  }
  ```

- **Description**: Sets the target waypoint.
- **Response**:
  - **Status Codes**:
    - `200`: Waypoint value set successfully
    - `400`: Invalid request

---

#### **Set Position P for PID (Channel 0)**

- **Endpoint URL**: `/pid/ch0/position/p`
- **Method**: POST
- **Headers**: `Content-Type: application/json`
- **Request Body**:

  ```json
  {
    "value": float
  }
  ```

- **Description**: Sets the P value for position PID of channel 0.
- **Response**:
  - **Status Codes**:
    - `200`: PID value set successfully
    - `400`: Invalid request

---

#### **Set Position I for PID (Channel 0)**

- **Endpoint URL**: `/pid/ch0/position/i`
- **Method**: POST
- **Headers**: `Content-Type: application/json`
- **Request Body**:

  ```json
  {
    "value": float
  }
  ```

- **Description**: Sets the I value for position PID of channel 0.
- **Response**:
  - **Status Codes**:
    - `200`: PID value set successfully
    - `400`: Invalid request

---

#### **Set Position D for PID (Channel 0)**

- **Endpoint URL**: `/pid/ch0/position/d`
- **Method**: POST
- **Headers**: `Content-Type: application/json`
- **Request Body**:

  ```json
  {
    "value": float
  }
  ```

- **Description**: Sets the D value for position PID of channel 0.
- **Response**:
  - **Status Codes**:
    - `200`: PID value set successfully
    - `400`: Invalid request

---

#### **Set Velocity P for PID (Channel 0)**

- **Endpoint URL**: `/pid/ch0/velocity/p`
- **Method**: POST
- **Headers**: `Content-Type: application/json`
- **Request Body**:

  ```json
  {
    "value": float
  }
  ```

- **Description**: Sets the P value for velocity PID of channel 0.
- **Response**:
  - **Status Codes**:
    - `200`: PID value set successfully
    - `400`: Invalid request

---

#### **Set Velocity I for PID (Channel 0)**

- **Endpoint URL**: `/pid/ch0/velocity/i`
- **Method**: POST
- **Headers**: `Content-Type: application/json`
- **Request Body**:

  ```json
  {
    "value": float
  }
  ```

- **Description**: Sets the I value for velocity PID of channel 0.
- **Response**:
  - **Status Codes**:
    - `200`: PID value set successfully
    - `400`: Invalid request

---

#### **Set Velocity D for PID (Channel 0)**

- **Endpoint URL**: `/pid/ch0/velocity/d`
- **Method**: POST
- **Headers**: `Content-Type: application/json`
- **Request Body**:

  ```json
  {
    "value": float
  }
  ```

- **Description**: Sets the D value for velocity PID of channel 0.
- **Response**:
  - **Status Codes**:
    - `200`: PID value set successfully
    - `400`: Invalid request

---

#### **Set Position P for PID (Channel 1)**

- **Endpoint URL**: `/pid/ch0/position/p`
- **Method**: POST
- **Headers**: `Content-Type: application/json`
- **Request Body**:

  ```json
  {
    "value": float
  }
  ```

- **Description**: Sets the P value for position PID of channel 1.
- **Response**:
  - **Status Codes**:
    - `200`: PID value set successfully
    - `400`: Invalid request

---

#### **Set Position I for PID (Channel 1)**

- **Endpoint URL**: `/pid/ch0/position/i`
- **Method**: POST
- **Headers**: `Content-Type: application/json`
- **Request Body**:

  ```json
  {
    "value": float
  }
  ```

- **Description**: Sets the I value for position PID of channel 1.
- **Response**:
  - **Status Codes**:
    - `200`: PID value set successfully
    - `400`: Invalid request

---

#### **Set Position D for PID (Channel 1)**

- **Endpoint URL**: `/pid/ch1/position/d`
- **Method**: POST
- **Headers**: `Content-Type: application/json`
- **Request Body**:

  ```json
  {
    "value": float
  }
  ```

- **Description**: Sets the D value for position PID of channel 1.
- **Response**:
  - **Status Codes**:
    - `200`: PID value set successfully
    - `400`: Invalid request

---

#### **Set Velocity P for PID (Channel 1)**

- **Endpoint URL**: `/pid/ch1/velocity/p`
- **Method**: POST
- **Headers**: `Content-Type: application/json`
- **Request Body**:

  ```json
  {
    "value": float
  }
  ```

- **Description**: Sets the P value for velocity PID of channel 1.
- **Response**:
  - **Status Codes**:
    - `200`: PID value set successfully
    - `400`: Invalid request

---

#### **Set Velocity I for PID (Channel 1)**

- **Endpoint URL**: `/pid/ch1/velocity/i`
- **Method**: POST
- **Headers**: `Content-Type: application/json`
- **Request Body**:

  ```json
  {
    "value": float
  }
  ```

- **Description**: Sets the I value for velocity PID of channel 1.
- **Response**:
  - **Status Codes**:
    - `200`: PID value set successfully
    - `400`: Invalid request

---

#### **Set Velocity D for PID (Channel 1)**

- **Endpoint URL**: `/pid/ch1/velocity/d`
- **Method**: POST
- **Headers**: `Content-Type: application/json`
- **Request Body**:

  ```json
  {
    "value": float
  }
  ```

- **Description**: Sets the D value for velocity PID of channel 1.
- **Response**:
  - **Status Codes**:
    - `200`: PID value set successfully
    - `400`: Invalid request

---
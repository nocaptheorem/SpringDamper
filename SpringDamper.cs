using Godot;
using System;
using System.Net.Sockets;
using System.Text;
using System.Text.Json;
using System.Linq;

// =================================================================================
// CONFIGURATION
// =================================================================================
public partial class SpringDamperConfig : Resource
{
    // Physics Parameters for F(t) = -kx - cx'
    public float BobMass { get; set; } = 50.0f;
    public float RestLength { get; set; } = 40.0f;
    public float SpringStiffness { get; set; } = 300.0f;
    public float SpringDamping { get; set; } = 1.0f;

    // Visuals
    public float BobRadius { get; set; } = 3.0f;
    public float CoilRadius { get; set; } = 3.0f;
    public float WireThickness { get; set; } = 0.3f;
    public int CoilCount { get; set; } = 12;
    public int SegmentsPerCoil { get; set; } = 100;

    // Camera Navigation
    public float CameraMoveSpeed { get; set; } = 50.0f;
    public float MouseSensitivity { get; set; } = 0.003f;


    // Telemetry (Network Layer)
    public bool EnableTelemetry { get; set; } = true;
    public float TelemetryPrintRateHz { get; set; } = 10.0f;
    public string UdpIp { get; set; } = "127.0.0.1";
    public int UdpPort { get; set; } = 9870;
}

// =================================================================================
// FREE CAMERA LAYER
// =================================================================================
public partial class FreeCamera : Camera3D
{
    private SpringDamperConfig _config;
    private Vector3 _camRotation = Vector3.Zero;

    public void Initialize(SpringDamperConfig config)
    {
        _config = config;
        Input.MouseMode = Input.MouseModeEnum.Captured;
    }

    public override void _Input(InputEvent @event)
    {
        // Toggle Mouse Capture with ESC
        if (@event.IsActionPressed("ui_cancel"))
        {
            Input.MouseMode = (Input.MouseMode == Input.MouseModeEnum.Captured)
                ? Input.MouseModeEnum.Visible
                : Input.MouseModeEnum.Captured;
        }

        // Camera Rotation (Mouse Look)
        if (@event is InputEventMouseMotion m && Input.MouseMode == Input.MouseModeEnum.Captured)
        {
            _camRotation.Y -= m.Relative.X * _config.MouseSensitivity;
            _camRotation.X -= m.Relative.Y * _config.MouseSensitivity;
            _camRotation.X = Mathf.Clamp(_camRotation.X, -1.5f, 1.5f);
            Rotation = new Vector3(_camRotation.X, _camRotation.Y, 0);
        }
    }

    public override void _Process(double delta)
    {
        if (Input.MouseMode != Input.MouseModeEnum.Captured) return;

        Vector3 dir = Vector3.Zero;
        float speed = _config.CameraMoveSpeed * (Input.IsKeyPressed(Key.Shift) ? 2.5f : 1.0f);

        if (Input.IsKeyPressed(Key.W)) dir.Z -= 1;
        if (Input.IsKeyPressed(Key.S)) dir.Z += 1;
        if (Input.IsKeyPressed(Key.A)) dir.X -= 1;
        if (Input.IsKeyPressed(Key.D)) dir.X += 1;
        if (Input.IsKeyPressed(Key.Space)) dir.Y += 1;
        if (Input.IsKeyPressed(Key.Ctrl)) dir.Y -= 1;

        if (dir.LengthSquared() > 0) dir = dir.Normalized();

        GlobalPosition += (GlobalBasis * dir) * speed * (float)delta;
    }
}

// =================================================================================
// PROCEDURAL SPRING RENDERER
// =================================================================================
public partial class CoilRenderer : Node3D
{
    private MultiMeshInstance3D _multiMeshInst;
    private SpringDamperConfig _config;
    private int _totalSegments;

    public void Initialize(SpringDamperConfig config)
    {
        _config = config;
        _totalSegments = config.CoilCount * config.SegmentsPerCoil;

        _multiMeshInst = new MultiMeshInstance3D
        {
            CastShadow = GeometryInstance3D.ShadowCastingSetting.Off,
            Multimesh = new MultiMesh
            {
                TransformFormat = MultiMesh.TransformFormatEnum.Transform3D,
                InstanceCount = _totalSegments,
                Mesh = new CylinderMesh
                {
                    TopRadius = config.WireThickness,
                    BottomRadius = config.WireThickness,
                    Height = 1.0f,
                    RadialSegments = 6
                }
            },
            MaterialOverride = new StandardMaterial3D
            {
                AlbedoColor = Colors.Yellow,
                ShadingMode = StandardMaterial3D.ShadingModeEnum.PerPixel
            }
        };
        AddChild(_multiMeshInst);
    }

    public void UpdateSpringVisuals(Vector3 start, Vector3 end)
    {
        Vector3 springAxis = end - start;
        float currentLength = springAxis.Length();
        if (currentLength < 0.001f) return;

        Vector3 direction = springAxis.Normalized();

        Vector3 refAxis = Mathf.Abs(direction.Dot(Vector3.Up)) > 0.99f ? Vector3.Right : Vector3.Up;
        Vector3 u = direction.Cross(refAxis).Normalized();
        Vector3 v = u.Cross(direction).Normalized();

        Vector3 previousPoint = GetCoilPoint(start, direction, u, v, currentLength, 0);

        for (int i = 0; i < _totalSegments; i++)
        {
            float t = (float)(i + 1) / _totalSegments;
            Vector3 currentPoint = GetCoilPoint(start, direction, u, v, currentLength, t);

            OrientSegment(i, previousPoint, currentPoint);
            previousPoint = currentPoint;
        }
    }

    private Vector3 GetCoilPoint(Vector3 start, Vector3 dir, Vector3 u, Vector3 v, float length, float t)
    {
        Vector3 centerAxisPoint = start + (dir * length * t);
        float angle = t * _config.CoilCount * Mathf.Tau;
        return centerAxisPoint + (u * Mathf.Cos(angle) * _config.CoilRadius) + (v * Mathf.Sin(angle) * _config.CoilRadius);
    }

    private void OrientSegment(int index, Vector3 start, Vector3 end)
    {
        Vector3 vector = end - start;
        if (vector.LengthSquared() < 0.0001f) return;

        Vector3 yAxis = vector;
        Vector3 direction = yAxis.Normalized();
        Vector3 refAxis = Mathf.Abs(direction.Dot(Vector3.Up)) > 0.99f ? Vector3.Right : Vector3.Up;
        Vector3 xAxis = direction.Cross(refAxis).Normalized();
        Vector3 zAxis = xAxis.Cross(direction).Normalized();

        Basis basis = new Basis(xAxis, yAxis, zAxis);
        Vector3 midPoint = start + (vector * 0.5f);

        _multiMeshInst.Multimesh.SetInstanceTransform(index, new Transform3D(basis, midPoint));
    }
}

// =================================================================================
// ORCHESTRATOR
// =================================================================================
public partial class SpringDamper : Node3D
{
    private SpringDamperConfig _config = new SpringDamperConfig();
    private CoilRenderer _renderer;
    private FreeCamera _camera;

    private RigidBody3D _anchor;
    private RigidBody3D _bob;

    // Telemetry State
    private UdpClient _udpClient;
    private float _telemetryTimer = 0.0f;
    private float _lastUdpErrorTime = -10.0f;
    private const float ERROR_LOG_INTERVAL_SEC = 2.0f;
    private const float Gravity = 9.8f;
    private float _lastAppliedForceMag = 0.0f;

    public override void _Ready()
    {
        _renderer = new CoilRenderer();
        AddChild(_renderer);
        _renderer.Initialize(_config);

        SetupLighting();
        CreateCamera();
        BuildPhysicsNodes();

        // Initialize Telemetry
        if (_config.EnableTelemetry)
        {
          try
          {
            _udpClient = new UdpClient();
          }
          catch (Exception ex)
          {
            GD.PrintErr($"[TELEMETRY INIT ERROR] Failed to instantiate UdpClient: {ex.Message}");
          }
        }
    }

    public override void _PhysicsProcess(double delta)
    {
        ApplyHarmonicOscillatorForces();

        // Process Telemetry
        if (_config.EnableTelemetry)
        {
          _telemetryTimer += (float)delta;
          if (_config.TelemetryPrintRateHz > 0 && _telemetryTimer >= (1.0f / _config.TelemetryPrintRateHz))
          {
            LogTelemetry();
            _telemetryTimer = 0f;
          }
        }
    }

    public override void _Process(double delta)
    {
        if (_anchor != null && _bob != null)
        {
            _renderer.UpdateSpringVisuals(_anchor.GlobalPosition, _bob.GlobalPosition);
        }
    }

    public override void _ExitTree()
    {
      _udpClient?.Close();
      _udpClient?.Dispose();
      _udpClient = null;
    }

    private void ApplyHarmonicOscillatorForces()
    {
        Vector3 springVector = _bob.GlobalPosition - _anchor.GlobalPosition;
        float currentDistance = springVector.Length();

        if (currentDistance < 0.001f) return;

        Vector3 springDir = springVector.Normalized();

        float displacement = currentDistance - _config.RestLength;
        float velocityAlongSpring = _bob.LinearVelocity.Dot(springDir);

        float springForce = -(_config.SpringStiffness * displacement);
        float damperForce = -(_config.SpringDamping * velocityAlongSpring);

        float totalForce = springForce + damperForce;
        Vector3 forceVector = springDir * totalForce;
        _lastAppliedForceMag = forceVector.Length();

        _bob.ApplyCentralForce(forceVector);
    }

    private void LogTelemetry()
    {
      if (_udpClient == null)
      {
        ReportTelemetryError("UdpClient is uninitialized or null.");
        return;
      }

      float anchorY = _anchor != null ? _anchor.GlobalPosition.Y : 0.0f;

      // 1. Calculate Kinetic Energy
      float kineticEnergy = 0.5f * _bob.Mass * _bob.LinearVelocity.LengthSquared();

      // 2. Calculate Gravitational Potential Energy
      float gravitationalPotential = _bob.Mass * Gravity * (_bob.GlobalPosition.Y - anchorY);

      // 3. Calculate Elastic Potential Energy (1/2 k x^2)
      float currentDistance = (_bob.GlobalPosition - _anchor.GlobalPosition).Length();
      float displacement = currentDistance - _config.RestLength;
      float elasticPotential = 0.5f * _config.SpringStiffness * (displacement * displacement);

      // Total Potential Energy includes both for a proper Lagrangian
      float totalPotential = gravitationalPotential + elasticPotential;

      var bobState = new
      {
        id = 1,
        mass = _bob.Mass,
        pos = new { x = _bob.GlobalPosition.X, y = _bob.GlobalPosition.Y, z = _bob.GlobalPosition.Z },
        vel = new { x = _bob.LinearVelocity.X, y = _bob.LinearVelocity.Y, z = _bob.LinearVelocity.Z },
        kinetic_energy = kineticEnergy,
        potential_energy = totalPotential
      };

      var metrics = new
      {
        timestamp = Time.GetTicksMsec() / 1000.0f,
        bob_count = 1,
        total_kinetic_energy = kineticEnergy,
        total_potential_energy = totalPotential,
        total_mechanical_energy = kineticEnergy + totalPotential,
        lagrangian = kineticEnergy - totalPotential,
        applied_spring_force_n = _lastAppliedForceMag,
        bobs = new[] { bobState }
      };

      try
      {
        string jsonString = JsonSerializer.Serialize(metrics);
        byte[] payload = Encoding.UTF8.GetBytes(jsonString);
        _udpClient.Send(payload, payload.Length, _config.UdpIp, _config.UdpPort);
      }
      catch (SocketException ex)
      {
        ReportTelemetryError($"SocketException on port {_config.UdpPort}: {ex.Message} (Code: {ex.SocketErrorCode})");
      }
      catch (Exception ex)
      {
        ReportTelemetryError($"Unexpected telemetry serialization/transmission error: {ex.Message}");
      }
    }

    private void ReportTelemetryError(string message)
    {
      float currentTime = Time.GetTicksMsec() / 1000.0f;
      if (currentTime - _lastUdpErrorTime >= ERROR_LOG_INTERVAL_SEC)
      {
        GD.PrintErr($"[TELEMETRY ERROR] {message}");
        _lastUdpErrorTime = currentTime;
      }
    }

    private void BuildPhysicsNodes()
    {
        _anchor = new RigidBody3D { Freeze = true };
        AddChild(_anchor);
        CreateMeshShape(_anchor, Colors.Gray, 3.0f);

        _bob = new RigidBody3D
        {
            Mass = _config.BobMass,
            Position = new Vector3(0, -(_config.RestLength * 0.5f), 0),
            LinearDampMode = RigidBody3D.DampMode.Replace,
            AngularDampMode = RigidBody3D.DampMode.Replace,
            LinearDamp = 0.0f,
            AngularDamp = 0.0f
        };
        AddChild(_bob);
        CreateMeshShape(_bob, Colors.Red, _config.BobRadius);
    }

    private void CreateMeshShape(RigidBody3D body, Color color, float radius)
    {
        var meshInstance = new MeshInstance3D();
        meshInstance.Mesh = new SphereMesh { Radius = radius, Height = radius * 2 };

        meshInstance.MaterialOverride = new StandardMaterial3D
        {
            AlbedoColor = color,
            Roughness = 0.2f,
            Metallic = 0.8f
        };
        body.AddChild(meshInstance);

        var col = new CollisionShape3D { Shape = new SphereShape3D { Radius = radius } };
        body.AddChild(col);
    }

    private void SetupLighting()
    {
        var env = new Godot.Environment
        {
            BackgroundMode = Godot.Environment.BGMode.Color,
            BackgroundColor = new Color(0.04f, 0.04f, 0.06f),
            TonemapMode = Godot.Environment.ToneMapper.Aces,
            TonemapExposure = 1.0f,
            AmbientLightSource = Godot.Environment.AmbientSource.Color,
            AmbientLightColor = new Color(0.12f, 0.15f, 0.22f),
            AmbientLightEnergy = 0.6f,
            SsaoEnabled = true,
            SsaoRadius = 1.5f,
            SsaoIntensity = 2.0f,
            GlowEnabled = true,
            GlowIntensity = 0.5f,
            GlowBloom = 0.15f,
            GlowBlendMode = Godot.Environment.GlowBlendModeEnum.Additive
        };
        AddChild(new WorldEnvironment { Environment = env });

        AddChild(new DirectionalLight3D { RotationDegrees = new Vector3(-45, 35, 0), LightColor = new Color(1.0f, 0.95f, 0.88f), LightEnergy = 1.8f, ShadowEnabled = true, ShadowBlur = 1.5f });
        AddChild(new DirectionalLight3D { RotationDegrees = new Vector3(-20, -135, 0), LightColor = new Color(0.6f, 0.75f, 1.0f), LightEnergy = 0.5f, ShadowEnabled = false });
        AddChild(new DirectionalLight3D { RotationDegrees = new Vector3(-15, 180, 0), LightColor = new Color(0.9f, 0.95f, 1.0f), LightEnergy = 0.8f, ShadowEnabled = false });
    }

    private void CreateCamera()
    {
        _camera = new FreeCamera();
        AddChild(_camera);
        _camera.GlobalPosition = new Vector3(0, -25.0f, 90.0f);
        _camera.Initialize(_config);
    }
}

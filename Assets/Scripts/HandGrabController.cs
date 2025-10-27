using UnityEngine;
using UnityEngine.XR.Hands;
using UnityEngine.XR.Interaction.Toolkit;
using UnityEngine.XR.Interaction.Toolkit.Interactables;

public class HandGrabController : MonoBehaviour
{
    [Header("Hand Setup")]
    [Tooltip("Left or Right")]
    [SerializeField] private Handedness handedness = Handedness.Left;
    [SerializeField] private Transform palmTransform; // L_Palm veya R_Palm
    
    [Header("Grab Settings")]
    [SerializeField] private float grabRadius = 0.15f;
    [SerializeField] private float grabCurlThreshold = 0.5f;
    [SerializeField] private float releaseCurlThreshold = 0.3f;
    
    [Header("Throw Settings")]
    [SerializeField] private float throwVelocityScale = 1.8f;
    [SerializeField] private int velocitySamples = 5;
    
    [Header("Debug")]
    [SerializeField] private bool showGizmos = true;
    [SerializeField] private bool verboseLogging = true;
    [SerializeField] private float debugLogInterval = 1f;
    
    private XRHand xrHand;
    private XRGrabInteractable currentGrabbedObject;
    private bool isGrabbing = false;
    private Vector3[] velocityHistory;
    private Quaternion[] rotationHistory;
    private int historyIndex = 0;
    private float currentCurl = 0f;
    private float lastDebugLogTime = 0f;
    private bool handFound = false;
    
    void Start()
    {
        velocityHistory = new Vector3[velocitySamples];
        rotationHistory = new Quaternion[velocitySamples];
        
        Debug.Log($"✓ Hand Grab Controller initialized for {gameObject.name}");
        
        if (palmTransform == null)
            Debug.LogError($"❌ Palm Transform NOT assigned on {gameObject.name}!");
    }
    
    void Update()
    {
        // XR Hand'i her frame kontrol et (lazy initialization)
        if (!handFound)
        {
            TryFindXRHand();
            if (!handFound)
            {
                if (verboseLogging && Time.time - lastDebugLogTime > debugLogInterval * 3)
                {
                    Debug.LogWarning($"⚠️ {gameObject.name}: Still searching for XR Hand ({handedness})...");
                    lastDebugLogTime = Time.time;
                }
                return;
            }
        }
        
        if (!xrHand.isTracked)
        {
            if (verboseLogging && Time.time - lastDebugLogTime > debugLogInterval)
            {
                Debug.LogWarning($"⚠️ {gameObject.name}: XR Hand NOT tracked!");
                lastDebugLogTime = Time.time;
            }
            return;
        }
        
        UpdateVelocityHistory();
        CheckGrabConditions();
        
        if (verboseLogging && Time.time - lastDebugLogTime > debugLogInterval)
        {
            LogCurrentState();
            lastDebugLogTime = Time.time;
        }
    }
    
    void TryFindXRHand()
    {
        // XRHandSubsystem'den el al
        var subsystems = new System.Collections.Generic.List<XRHandSubsystem>();
        SubsystemManager.GetSubsystems(subsystems);
        
        if (subsystems.Count == 0)
        {
            return;
        }
        
        foreach (var subsystem in subsystems)
        {
            if (handedness == Handedness.Left)
            {
                xrHand = subsystem.leftHand;
            }
            else
            {
                xrHand = subsystem.rightHand;
            }
            
            if (xrHand != null)
            {
                handFound = true;
                Debug.Log($"✓ Found XR Hand ({handedness}) via subsystem!");
                break;
            }
        }
    }
    
    void LogCurrentState()
    {
        Debug.Log($"━━━ {gameObject.name} ({handedness}) Status ━━━");
        Debug.Log($"🤚 XR Hand Tracked: {xrHand.isTracked}");
        Debug.Log($"📍 Palm Position: {palmTransform.position}");
        Debug.Log($"👊 Hand Curl: {currentCurl:F2} (Threshold: {grabCurlThreshold:F2})");
        Debug.Log($"🎯 Is Grabbing: {isGrabbing}");
        
        Collider[] nearbyColliders = Physics.OverlapSphere(palmTransform.position, grabRadius);
        Debug.Log($"🔍 Nearby Colliders: {nearbyColliders.Length}");
        
        if (nearbyColliders.Length > 0)
        {
            foreach (var col in nearbyColliders)
            {
                var grabInteractable = col.GetComponent<XRGrabInteractable>();
                float distance = Vector3.Distance(palmTransform.position, col.transform.position);
                Debug.Log($"   - {col.gameObject.name} ({distance:F3}m) " +
                         $"[XRGrabInteractable: {(grabInteractable != null ? "✓" : "✗")}]");
            }
        }
        else
        {
            Debug.Log($"   ⚠️ No objects within {grabRadius}m radius!");
        }
    }
    
    void UpdateVelocityHistory()
    {
        if (palmTransform == null) return;
        
        velocityHistory[historyIndex] = palmTransform.position;
        rotationHistory[historyIndex] = palmTransform.rotation;
        historyIndex = (historyIndex + 1) % velocitySamples;
    }
    
    void CheckGrabConditions()
    {
        currentCurl = CalculateHandCurl();
        
        if (verboseLogging && Mathf.Abs(currentCurl - grabCurlThreshold) < 0.05f)
        {
            Debug.Log($"⚡ {gameObject.name} curl near threshold: {currentCurl:F2}");
        }
        
        if (!isGrabbing && currentCurl > grabCurlThreshold)
        {
            Debug.Log($"✅ {gameObject.name} curl > threshold! Trying to grab...");
            TryGrab();
        }
        else if (isGrabbing && currentCurl < releaseCurlThreshold)
        {
            Debug.Log($"✅ {gameObject.name} curl < release threshold! Releasing...");
            Release();
        }
    }
    
    void TryGrab()
    {
        Collider[] nearbyColliders = Physics.OverlapSphere(palmTransform.position, grabRadius);
        
        Debug.Log($"🔎 TryGrab: Found {nearbyColliders.Length} colliders in radius");
        
        if (nearbyColliders.Length == 0)
        {
            Debug.LogWarning($"❌ TryGrab FAILED: No colliders in {grabRadius}m radius!");
            return;
        }
        
        foreach (var col in nearbyColliders)
        {
            Debug.Log($"   Checking: {col.gameObject.name}");
            
            var grabInteractable = col.GetComponent<XRGrabInteractable>();
            
            if (grabInteractable == null)
            {
                Debug.Log($"      ✗ No XRGrabInteractable");
                continue;
            }
            
            if (!grabInteractable.enabled)
            {
                Debug.Log($"      ✗ XRGrabInteractable disabled");
                continue;
            }
            
            currentGrabbedObject = grabInteractable;
            isGrabbing = true;
            
            AttachObject(grabInteractable);
            
            Debug.Log($"✅✅✅ GRABBED: {grabInteractable.name} (curl: {currentCurl:F2}) ✅✅✅");
            return;
        }
        
        Debug.LogWarning($"❌ TryGrab FAILED: No valid XRGrabInteractable!");
    }
    
    void AttachObject(XRGrabInteractable interactable)
    {
        Debug.Log($"📎 Attaching {interactable.name} to {palmTransform.name}");
        
        var rb = interactable.GetComponent<Rigidbody>();
        if (rb != null)
        {
            rb.isKinematic = true;
        }
        
        interactable.transform.SetParent(palmTransform);
        Debug.Log($"   ✓ Attached!");
    }
    
    void Release()
    {
        if (currentGrabbedObject == null) return;
        
        Debug.Log($"🚀 Releasing: {currentGrabbedObject.name}");
        
        Vector3 throwVelocity = CalculateThrowVelocity();
        Vector3 throwAngularVelocity = CalculateAngularVelocity();
        
        currentGrabbedObject.transform.SetParent(null);
        
        var rb = currentGrabbedObject.GetComponent<Rigidbody>();
        if (rb != null)
        {
            rb.isKinematic = false;
            rb.linearVelocity = throwVelocity * throwVelocityScale;
            rb.angularVelocity = throwAngularVelocity;
        }
        
        Debug.Log($"✅ Released! Velocity: {(throwVelocity * throwVelocityScale).magnitude:F2} m/s");
        
        currentGrabbedObject = null;
        isGrabbing = false;
    }
    
    float CalculateHandCurl()
    {
        if (xrHand == null) return 0f;
        
        XRHandJointID[] proximalJoints = {
            XRHandJointID.IndexProximal,
            XRHandJointID.MiddleProximal,
            XRHandJointID.RingProximal
        };
        
        XRHandJointID[] tipJoints = {
            XRHandJointID.IndexTip,
            XRHandJointID.MiddleTip,
            XRHandJointID.RingTip
        };
        
        float totalCurl = 0f;
        int validFingers = 0;
        
        var wrist = xrHand.GetJoint(XRHandJointID.Wrist);
        if (!wrist.TryGetPose(out var wristPose)) return 0f;
        
        for (int i = 0; i < 3; i++)
        {
            var proximal = xrHand.GetJoint(proximalJoints[i]);
            var tip = xrHand.GetJoint(tipJoints[i]);
            
            if (proximal.TryGetPose(out var proximalPose) && 
                tip.TryGetPose(out var tipPose))
            {
                float tipToWrist = Vector3.Distance(tipPose.position, wristPose.position);
                float proximalToWrist = Vector3.Distance(proximalPose.position, wristPose.position);
                
                float curl = 1f - Mathf.Clamp01((tipToWrist - proximalToWrist) / proximalToWrist);
                
                totalCurl += curl;
                validFingers++;
            }
        }
        
        return validFingers > 0 ? totalCurl / validFingers : 0f;
    }
    
    Vector3 CalculateThrowVelocity()
    {
        Vector3 totalVelocity = Vector3.zero;
        
        for (int i = 0; i < velocitySamples - 1; i++)
        {
            int currentIndex = i;
            int nextIndex = (i + 1) % velocitySamples;
            
            Vector3 velocity = (velocityHistory[nextIndex] - velocityHistory[currentIndex]) / Time.fixedDeltaTime;
            totalVelocity += velocity;
        }
        
        return totalVelocity / (velocitySamples - 1);
    }
    
    Vector3 CalculateAngularVelocity()
    {
        Quaternion deltaRotation = rotationHistory[(historyIndex - 1 + velocitySamples) % velocitySamples] * 
                                    Quaternion.Inverse(rotationHistory[historyIndex]);
        
        deltaRotation.ToAngleAxis(out float angle, out Vector3 axis);
        
        return axis * (angle * Mathf.Deg2Rad / Time.fixedDeltaTime);
    }
    
    void OnDrawGizmos()
    {
        if (!showGizmos) return;
        if (palmTransform == null) return;
        
        Gizmos.color = isGrabbing ? Color.red : Color.green;
        Gizmos.DrawWireSphere(palmTransform.position, grabRadius);
        
        if (Application.isPlaying && currentCurl > 0)
        {
            Gizmos.color = Color.Lerp(Color.green, Color.red, currentCurl);
            Gizmos.DrawLine(palmTransform.position, palmTransform.position + Vector3.up * currentCurl * 0.2f);
        }
        
        Gizmos.color = Color.yellow;
        Gizmos.DrawLine(palmTransform.position, palmTransform.position + palmTransform.forward * 0.05f);
        
        if (Application.isPlaying)
        {
            Collider[] nearbyColliders = Physics.OverlapSphere(palmTransform.position, grabRadius);
            Gizmos.color = Color.cyan;
            foreach (var col in nearbyColliders)
            {
                Gizmos.DrawLine(palmTransform.position, col.transform.position);
            }
        }
    }
}
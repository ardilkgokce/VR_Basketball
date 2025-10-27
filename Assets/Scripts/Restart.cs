using UnityEngine;
using UnityEngine.InputSystem; 
using UnityEngine.SceneManagement;

public class Restart : MonoBehaviour
{
    private InputAction restartAction;

    void OnEnable()
    {
        restartAction = new InputAction(type: InputActionType.Button, binding: "<Keyboard>/space");
        restartAction.performed += OnRestart; // olay bağlama
        restartAction.Enable();
    }

    void OnDisable()
    {
        restartAction.Disable();
    }

    private void OnRestart(InputAction.CallbackContext context)
    {
        SceneManager.LoadScene(SceneManager.GetActiveScene().name);
        Debug.Log("Restart triggered!");
    }
}
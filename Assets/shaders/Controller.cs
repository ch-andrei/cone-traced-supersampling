using System;
using UnityEngine;

namespace shaders
{
    public class Controller : MonoBehaviour
    {
        private Material mat;

        void Awake()
        {
            this.mat = this.GetComponent<Renderer>().material;

            this.mat.SetVector("_MousePos", new Vector4(1f, 0f, 0f, 0f));
        }

        public static int GetUnixTime()
        {
            return (int) (DateTime.UtcNow - new DateTime(1970, 1, 1)).TotalSeconds;
        }

        void Update()
        {
            if (Input.GetMouseButton(0))
            {
                var pos = Input.mousePosition;
                this.mat.SetVector("_MousePos", new Vector4(1f - pos.x / Screen.width, pos.y / Screen.height, 0f, 0f));
                Debug.Log("mousePosition " + pos);
            }

            if (Input.GetKeyDown(KeyCode.S))
            {
                string path = "screenshot-" + GetUnixTime() + ".png";
                ScreenCapture.CaptureScreenshot(path);
                Debug.Log("Saved screenshot " + path);
            }

            if (Input.GetKeyDown(KeyCode.R))
            {
                Vector3 pos = new Vector3(0f, 0f, 0f);
                this.mat.SetVector("_MousePos", new Vector4(1f - pos.x / Screen.width, pos.y / Screen.height, 0f, 0f));
                Debug.Log("Reset mouse position to default.");
            }
        }
    }
}

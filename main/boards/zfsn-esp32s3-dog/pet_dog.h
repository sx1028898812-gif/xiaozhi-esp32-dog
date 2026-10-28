#ifndef _PET_DOG_H_
#define _PET_DOG_H_

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <freertos/queue.h>
#include <driver/ledc.h>
#include <cstdint>
#include <string>
#include <map>

enum class DogActionType {
    kStand = 0,
    kSitdown,
    kSleep,
    kWalkForward,
    kWalkBack,
    kTurnLeft,
    kTurnRight,
    kStretch,
    kScratching,
    kSwing,
    kPetWave,
    kJump,
    kCustomKeyframe,
};

struct DogActionParams {
    DogActionType type;
    int steps;
    int angle;
    // Used when type == kCustomKeyframe
    int lf;
    int rf;
    int lb;
    int rb;
    int delay_ms;
    bool hold;
};

class PetDog {
public:
    static PetDog& GetInstance() {
        static PetDog instance;
        return instance;
    }

    PetDog(const PetDog&) = delete;
    PetDog& operator=(const PetDog&) = delete;

    // 13 Standard Actions (compatible with 2.1 and 6.0 MCP schema)
    void Stand();
    void Sitdown();
    void Sleep();
    void WalkForward(int steps = 4);
    void WalkBack(int steps = 4);
    void TurnLeft(int steps = 4);
    void TurnRight(int steps = 4);
    void Stretch();
    void Scratching();
    void Swing(int steps = 5, int angle = 30);
    void PetWave();
    void Jump();
    void Stop();

    // Random interactive action (for head touch long-press & idle free-explore)
    void ExecuteRandomAction();

    // Custom AI choreography sequence (JSON string with array of keyframes)
    bool ExecuteServoSequence(const std::string& sequence_json, std::string& error_msg);

    // NVS Servo Trim Calibration (-30 to +30 degrees)
    bool SetTrim(const std::string& servo_type, int trim_value);
    std::map<std::string, int> GetTrims() const;

    // Persistent Settings (NVS namespace "petdog")
    bool GetFreeExplore() const { return free_explore_; }
    void SetFreeExplore(bool enable);
    bool ToggleFreeExplore();

    bool GetBootAnimation() const { return boot_animation_; }
    void SetBootAnimation(bool enable);
    bool ToggleBootAnimation();

    bool GetEmotionSync() const { return emotion_sync_; }
    void SetEmotionSync(bool enable);
    bool ToggleEmotionSync();

    bool IsBusy() const { return is_busy_; }
    DogActionType GetCurrentPosture() const { return current_posture_; }

    // Direct angle setters (with trim compensation)
    void SetLeftFrontAngle(int angle);
    void SetRightFrontAngle(int angle);
    void SetLeftBackAngle(int angle);
    void SetRightBackAngle(int angle);
    void ReleaseServos();

private:
    PetDog();
    ~PetDog();

    void InitPwm();
    void LoadSettings();
    void QueueAction(const DogActionParams& params, bool clear_existing = false);
    bool WaitOrStop(int duration_ms);

    // Internal synchronous execution routines run inside ActionTask
    void RunStand();
    void RunSitdown();
    void RunSleep();
    void RunWalkForward(int steps);
    void RunWalkBack(int steps);
    void RunTurnLeft(int steps);
    void RunTurnRight(int steps);
    void RunStretch();
    void RunScratching();
    void RunSwing(int steps, int angle);
    void RunPetWave();
    void RunJump();

    static void ActionTaskTrampoline(void* arg);
    void ActionTaskLoop();

    QueueHandle_t action_queue_ = nullptr;
    EventGroupHandle_t event_group_ = nullptr;
    TaskHandle_t action_task_handle_ = nullptr;

    bool is_busy_ = false;
    DogActionType current_posture_ = DogActionType::kStand;

    // Servo Trim Offsets (degrees)
    int trim_lf_ = 0;
    int trim_rf_ = 0;
    int trim_lb_ = 0;
    int trim_rb_ = 0;

    // Persistent feature flags
    bool free_explore_ = false;
    bool boot_animation_ = true;
    bool emotion_sync_ = true;
};

#endif // _PET_DOG_H_


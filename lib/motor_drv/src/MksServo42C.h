#ifndef MKS_SERVO42C_H
#define MKS_SERVO42C_H

class MksServo42CPair {
public:
    bool begin();
    void enable();
    void disable();
    bool isEnabled() const return _enabled;
private:
    bool _initialized = false;
    bool _enabled = false;
};

#endif
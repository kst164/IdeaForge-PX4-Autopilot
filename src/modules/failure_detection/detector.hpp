#include "stdio.h"
#include <cstdint>
#include <cstdlib>
#include <math.h>
#include <stdlib.h>

namespace detector {
const int N = 1000;

struct dequeue {
    float data[N];
    int front, rear;
    int size;
};

struct dequeue create_dequeue() {
    struct dequeue dq;
    dq.front = -1;
    dq.rear = -1;
    dq.size = 0;

    return dq;
}

bool is_empty(struct dequeue dq) {
    return dq.size == 0;
}

bool is_full(struct dequeue dq) {
    return dq.size == N;
}

void pop_front(struct dequeue& dq) {
    if (is_empty(dq)) {
        dq.front = -1;
        dq.rear = -1;
        return;
    }

    dq.front = (dq.front + 1) % N;
    dq.size--;
}

void push_back(struct dequeue& dq, float x) {
    if (is_full(dq)) {
        pop_front(dq);
    }

    if (is_empty(dq)) {
        dq.front = 0;
        dq.rear = 0;
    } else {
        dq.rear = (dq.rear + 1) % N;
    }

    dq.data[dq.rear] = x;
    dq.size++;

    return;
}

float gradient(float x1, float y1, float x2, float y2) {
    return (y2 - y1) / (x2 - x1);
}

float mean(float x[], int n) {
    float sum = 0;
    for (int i = 0; i < n; i++) {
        sum += x[i];
    }

    return sum / n;
}

float mean_abs(float x[], int n) {
    float sum = 0;
    for (int i = 0; i < n; i++) {
        sum += abs(x[i]);
    }

    return sum / n;
}


class Detector {
public:
    struct dequeue dq_s[6];
    struct dequeue dq_t[6];
    int count[6] = {0};
    int pos_count;
    int neg_count;
    int max_count;
    const float threshold_grads[6] = {0.2, 0.2, 10, 0.01, 0.01, 0.01};
    const float threshold_vals[6] = {10, 10, 10, 100, 100, 2};

    Detector() {
        for (int i = 0; i < 6; i++) {
            dq_s[i] = create_dequeue();
            dq_t[i] = create_dequeue();
        }

        pos_count = 0;
        neg_count = 0;
        max_count = 10;
    }

    uint8_t check_failure_now(float signal, float time, int index) {
        push_back(dq_s[index], signal);
        push_back(dq_t[index], time);

        if (!is_full(dq_s[index])) {
            return 0;
        }

        float threshold_grad = threshold_grads[index];
        float threshold_val = threshold_vals[index];


        float mean_value_abs = mean_abs(dq_s[index].data, N);
        float mean_grad_abs = gradient(0, abs(dq_s[index].data[dq_s[index].front]), N - 1, abs(dq_s[index].data[dq_s[index].rear]));
        float curr_grad_abs = gradient(dq_t[index].data[(N + dq_t[index].rear-1) % N], abs(dq_s[index].data[(N + dq_s[index].rear-1) % N]), dq_t[index].data[dq_t[index].rear], abs(dq_s[index].data[dq_s[index].rear]));
        float curr_grad = gradient(dq_t[index].data[(N + dq_t[index].rear-1) % N], dq_s[index].data[(N + dq_s[index].rear-1) % N], dq_t[index].data[dq_t[index].rear], dq_s[index].data[dq_s[index].rear]);

        if (abs(curr_grad_abs/mean_grad_abs) > threshold_grad && abs(dq_s[index].data[dq_s[index].rear]/mean_value_abs) > threshold_val) {
            count[index]++;
            if (curr_grad > 0) {
                pos_count++;
            } else {
                neg_count++;
            }
        } else {
            count[index] = 0;
            pos_count = 0;
            neg_count = 0;
        }

        if (count[index] == max_count) {
            count[index] = 0;
            return 1;
        }

        return 0;
    }

    uint8_t check_failure_now(float signals[6], float time) {
        uint8_t count_ = 0;

        for (int i = 0; i < 6; i++) {
            count_ += check_failure_now(signals[i], time, i);
        }

        if (count_ >= 2) {
            return 1;
        }

        return 0;
    }

};
}

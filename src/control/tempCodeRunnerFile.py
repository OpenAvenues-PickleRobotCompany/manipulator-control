def compute_command(self, desired_state: float, current_state: float):
        error = desired_state - current_state
        ep = desired_state - self.last_output
        self.integral += error * self.ts + (1/self.T_t)*ep
        derivative = (error - self.last_error) / self.ts

        p = self.kp * error
        i = self.ki * self.integral
        d = self.kd * derivative

        command = p + i + d
        if command > self.max_output:
            aw_term = self.kp * (command - self.max_output)
            self.integral -= aw_term / self.ki + (1/self.T_t) * ep
            command = p + self.integral + d
            if command > self.max_output:
                command = self.max_output

        elif command < self.min_output:
            aw_term = self.kp * (command - self.min_output)
            self.integral -= (aw_term / self.ki) + (1/self.T_t) * ep
            command = self.min_output
            if command < self.min_output:
                command = self.min_output

        self.last_error = error

        return command
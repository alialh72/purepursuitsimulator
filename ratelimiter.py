class limiter:
    rateLimiterOutput = 0
    prevRateLimiterOutput = 0
    
    def rateLimiter(self, val, maxRate):
      maxChange = 0.02 * maxRate
      self.rateLimiterOutput += clamp(val - self.prevRateLimiterOutput, -maxChange, maxChange)
      self.prevRateLimiterOutput = self.rateLimiterOutput
      return self.rateLimiterOutput
    

    def reset(self) :
      self.rateLimiterOutput = 0
      self.prevRateLimiterOutput = 0
    

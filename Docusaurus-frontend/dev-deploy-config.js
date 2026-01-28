/**
 * Development and Deployment Configuration for Frontend-Backend Integration
 * This file contains configuration settings for local development and deployment scenarios
 */

// Configuration for different environments
const envConfig = {
  development: {
    // Backend API configuration for development
    BACKEND_URL: process.env.REACT_APP_BACKEND_URL || 'http://localhost:8000',

    // Frontend development settings
    FRONTEND_PORT: process.env.PORT || 3000,
    FRONTEND_HOST: 'localhost',

    // API request settings
    REQUEST_TIMEOUT: 30000, // 30 seconds
    RETRY_ATTEMPTS: 3,
    RETRY_DELAY: 1000, // Base delay in ms

    // Feature flags for development
    FEATURES: {
      ENABLE_SELECTED_TEXT_CONTEXT: true,
      ENABLE_ADAPTIVE_PROMPTS: true,
      ENABLE_CONFIDENCE_VISUALIZATION: true,
      ENABLE_MULTI_TURN_CONVERSATIONS: true,
      ENABLE_SOURCE_CITATIONS: true,
      ENABLE_TYPING_INDICATORS: true,
      ENABLE_ERROR_RETRY: true,
      ENABLE_DEBUG_LOGGING: true
    },

    // Performance thresholds for development
    PERFORMANCE: {
      SLOW_RESPONSE_THRESHOLD_MS: 5000, // Log warnings for responses slower than this
      MAX_CONTENT_LENGTH_KB: 2000, // Max selected text length in KB
      CONCURRENT_REQUESTS_LIMIT: 5
    }
  },

  production: {
    // Backend API configuration for production
    BACKEND_URL: process.env.REACT_APP_BACKEND_URL || 'https://your-production-backend.com',

    // Frontend production settings
    FRONTEND_PORT: process.env.PORT || 80,
    FRONTEND_HOST: '0.0.0.0',

    // API request settings
    REQUEST_TIMEOUT: 30000, // 30 seconds
    RETRY_ATTEMPTS: 2,
    RETRY_DELAY: 1500, // Base delay in ms

    // Feature flags for production
    FEATURES: {
      ENABLE_SELECTED_TEXT_CONTEXT: true,
      ENABLE_ADAPTIVE_PROMPTS: true,
      ENABLE_CONFIDENCE_VISUALIZATION: true,
      ENABLE_MULTI_TURN_CONVERSATIONS: true,
      ENABLE_SOURCE_CITATIONS: true,
      ENABLE_TYPING_INDICATORS: true,
      ENABLE_ERROR_RETRY: true,
      ENABLE_DEBUG_LOGGING: false // Disable debug logging in production
    },

    // Performance thresholds for production
    PERFORMANCE: {
      SLOW_RESPONSE_THRESHOLD_MS: 3000, // Log warnings for responses slower than this
      MAX_CONTENT_LENGTH_KB: 2000, // Max selected text length in KB
      CONCURRENT_REQUESTS_LIMIT: 3
    }
  },

  test: {
    // Configuration for testing environment
    BACKEND_URL: process.env.REACT_APP_BACKEND_URL || 'http://localhost:8000',

    // Testing settings
    FRONTEND_PORT: 3001, // Different port to avoid conflicts
    FRONTEND_HOST: 'localhost',

    // API request settings for tests
    REQUEST_TIMEOUT: 10000, // Shorter timeout for tests
    RETRY_ATTEMPTS: 1, // No retries in tests
    RETRY_DELAY: 500,

    // Feature flags for testing
    FEATURES: {
      ENABLE_SELECTED_TEXT_CONTEXT: true,
      ENABLE_ADAPTIVE_PROMPTS: true,
      ENABLE_CONFIDENCE_VISUALIZATION: true,
      ENABLE_MULTI_TURN_CONVERSATIONS: true,
      ENABLE_SOURCE_CITATIONS: true,
      ENABLE_TYPING_INDICATORS: true,
      ENABLE_ERROR_RETRY: false, // Disable retries in tests
      ENABLE_DEBUG_LOGGING: true
    },

    // Performance thresholds for testing
    PERFORMANCE: {
      SLOW_RESPONSE_THRESHOLD_MS: 10000, // Higher threshold for CI environments
      MAX_CONTENT_LENGTH_KB: 2000,
      CONCURRENT_REQUESTS_LIMIT: 1 // Single request for tests
    }
  }
};

/**
 * Get configuration based on current environment
 * @returns {Object} Configuration object for the current environment
 */
function getConfig() {
  const environment = process.env.NODE_ENV || 'development';
  const config = envConfig[environment];

  if (!config) {
    console.warn(`Environment '${environment}' not found, using development config`);
    return envConfig.development;
  }

  return config;
}

/**
 * Validate configuration settings
 * @param {Object} config - Configuration object to validate
 * @returns {Array} List of validation errors
 */
function validateConfig(config) {
  const errors = [];

  // Validate backend URL
  try {
    new URL(config.BACKEND_URL);
  } catch (e) {
    errors.push(`Invalid BACKEND_URL: ${config.BACKEND_URL}`);
  }

  // Validate numeric settings
  if (typeof config.REQUEST_TIMEOUT !== 'number' || config.REQUEST_TIMEOUT <= 0) {
    errors.push(`REQUEST_TIMEOUT must be a positive number, got: ${config.REQUEST_TIMEOUT}`);
  }

  if (typeof config.RETRY_ATTEMPTS !== 'number' || config.RETRY_ATTEMPTS < 0) {
    errors.push(`RETRY_ATTEMPTS must be a non-negative number, got: ${config.RETRY_ATTEMPTS}`);
  }

  if (typeof config.RETRY_DELAY !== 'number' || config.RETRY_DELAY <= 0) {
    errors.push(`RETRY_DELAY must be a positive number, got: ${config.RETRY_DELAY}`);
  }

  // Validate performance settings
  if (typeof config.PERFORMANCE.SLOW_RESPONSE_THRESHOLD_MS !== 'number' || config.PERFORMANCE.SLOW_RESPONSE_THRESHOLD_MS <= 0) {
    errors.push(`SLOW_RESPONSE_THRESHOLD_MS must be a positive number, got: ${config.PERFORMANCE.SLOW_RESPONSE_THRESHOLD_MS}`);
  }

  if (typeof config.PERFORMANCE.MAX_CONTENT_LENGTH_KB !== 'number' || config.PERFORMANCE.MAX_CONTENT_LENGTH_KB <= 0) {
    errors.push(`MAX_CONTENT_LENGTH_KB must be a positive number, got: ${config.PERFORMANCE.MAX_CONTENT_LENGTH_KB}`);
  }

  return errors;
}

/**
 * Log configuration summary (only in development)
 */
function logConfig() {
  const config = getConfig();
  const environment = process.env.NODE_ENV || 'development';

  if (environment === 'development' && config.FEATURES.ENABLE_DEBUG_LOGGING) {
    console.log('=== Frontend-Backend Integration Configuration ===');
    console.log(`Environment: ${environment}`);
    console.log(`Backend URL: ${config.BACKEND_URL}`);
    console.log(`Frontend Host: ${config.FRONTEND_HOST}:${config.FRONTEND_PORT}`);
    console.log(`Request Timeout: ${config.REQUEST_TIMEOUT}ms`);
    console.log(`Retry Attempts: ${config.RETRY_ATTEMPTS}`);
    console.log(`Features Enabled:`, Object.entries(config.FEATURES)
      .filter(([_, enabled]) => enabled)
      .map(([name, _]) => name)
      .join(', '));
    console.log('===============================================');
  }
}

// Export configuration functions and values
const config = getConfig();
const validationErrors = validateConfig(config);

// Log validation errors if any
if (validationErrors.length > 0) {
  console.error('Configuration validation errors:');
  validationErrors.forEach(error => console.error(`- ${error}`));
  throw new Error(`Configuration validation failed: ${validationErrors.join('; ')}`);
}

// Log configuration summary
logConfig();

module.exports = {
  config,
  getConfig,
  validateConfig,
  logConfig,
  envConfig
};
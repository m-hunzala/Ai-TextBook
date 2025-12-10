import React, { useState, useEffect } from 'react';
import { useAuth } from '../auth/authClient';
import { useUserData } from '../contexts/UserContext';

const AuthModal = ({ isOpen, onClose }) => {
  const [activeTab, setActiveTab] = useState('login'); // 'login' or 'signup'
  const [formData, setFormData] = useState({
    email: '',
    password: '',
    name: '', // For signup
    softwareBackground: '',
    hardwareAvailable: [],
    experienceLevel: ''
  });
  const [errors, setErrors] = useState({});
  const [isLoading, setIsLoading] = useState(false);

  const { signIn, signUp } = useAuth();
  const { isAuthenticated } = useUserData();

  // Close modal if user becomes authenticated
  useEffect(() => {
    if (isAuthenticated && isOpen) {
      onClose();
    }
  }, [isAuthenticated, isOpen, onClose]);

  // Handle form input changes
  const handleInputChange = (e) => {
    const { name, value } = e.target;
    setFormData(prev => ({
      ...prev,
      [name]: value
    }));

    // Clear error when user starts typing
    if (errors[name]) {
      setErrors(prev => {
        const newErrors = { ...prev };
        delete newErrors[name];
        return newErrors;
      });
    }
  };

  // Handle hardware selection (multi-select)
  const handleHardwareChange = (option) => {
    setFormData(prev => {
      const hardware = [...prev.hardwareAvailable];
      const index = hardware.indexOf(option);

      if (index >= 0) {
        hardware.splice(index, 1);
      } else {
        hardware.push(option);
      }

      return { ...prev, hardwareAvailable: hardware };
    });
  };

  // Validate form
  const validateForm = () => {
    const newErrors = {};

    // Email validation
    if (!formData.email.trim()) {
      newErrors.email = 'Email is required';
    } else if (!/\S+@\S+\.\S+/.test(formData.email)) {
      newErrors.email = 'Email is invalid';
    }

    // Password validation
    if (!formData.password.trim()) {
      newErrors.password = 'Password is required';
    } else if (formData.password.length < 6) {
      newErrors.password = 'Password must be at least 6 characters';
    }

    // Name validation (for signup only)
    if (activeTab === 'signup' && !formData.name.trim()) {
      newErrors.name = 'Name is required';
    }

    // Custom fields validation (for signup only)
    if (activeTab === 'signup') {
      if (!formData.softwareBackground.trim()) {
        newErrors.softwareBackground = 'Software background is required';
      }
      if (formData.hardwareAvailable.length === 0) {
        newErrors.hardwareAvailable = 'Please select at least one hardware option';
      }
      if (!formData.experienceLevel.trim()) {
        newErrors.experienceLevel = 'Experience level is required';
      }
    }

    setErrors(newErrors);
    return Object.keys(newErrors).length === 0;
  };

  // Handle form submission
  const handleSubmit = async (e) => {
    e.preventDefault();

    if (!validateForm()) {
      return;
    }

    setIsLoading(true);

    try {
      if (activeTab === 'login') {
        // Login
        const result = await signIn({
          email: formData.email,
          password: formData.password
        });

        if (result?.error) {
          setErrors({ general: result.error.message });
        } else {
          onClose(); // Close modal on successful login
        }
      } else {
        // Signup
        const result = await signUp({
          email: formData.email,
          password: formData.password,
          name: formData.name,
          softwareBackground: formData.softwareBackground,
          hardwareAvailable: formData.hardwareAvailable,
          experienceLevel: formData.experienceLevel
        });

        if (result?.error) {
          setErrors({ general: result.error.message });
        } else {
          onClose(); // Close modal on successful signup
        }
      }
    } catch (error) {
      console.error(activeTab === 'login' ? 'Login' : 'Signup', 'error:', error);
      setErrors({ general: `Failed to ${activeTab}. Please try again.` });
    } finally {
      setIsLoading(false);
    }
  };

  // Reset form when modal opens or tab changes
  useEffect(() => {
    if (isOpen) {
      setFormData({
        email: '',
        password: '',
        name: '',
        softwareBackground: '',
        hardwareAvailable: [],
        experienceLevel: ''
      });
      setErrors({});
    }
  }, [isOpen, activeTab]);

  if (!isOpen) {
    return null;
  }

  return (
    <div className="fixed inset-0 z-50 flex items-center justify-center p-4 bg-black bg-opacity-50 backdrop-blur-sm animate-fade-in">
      <div className="relative w-full max-w-md auth-modal-bg rounded-xl shadow-2xl overflow-hidden transform transition-all duration-300 scale-100 animate-slide-up">
        {/* Close Button */}
        <button
          onClick={onClose}
          className="absolute top-4 right-4 auth-modal-close-btn z-10 transition-colors duration-200"
          aria-label="Close"
        >
          <svg xmlns="http://www.w3.org/2000/svg" className="h-6 w-6" fill="none" viewBox="0 0 24 24" stroke="currentColor">
            <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M6 18L18 6M6 6l12 12" />
          </svg>
        </button>

        {/* Tabs */}
        <div className="flex border-b border-gray-200 dark:border-gray-700">
          <button
            className={`flex-1 py-4 px-6 text-center font-medium transition-colors duration-200 ${
              activeTab === 'login'
                ? 'auth-modal-tab-active'
                : 'auth-modal-tab-inactive'
            }`}
            onClick={() => setActiveTab('login')}
          >
            Login
          </button>
          <button
            className={`flex-1 py-4 px-6 text-center font-medium transition-colors duration-200 ${
              activeTab === 'signup'
                ? 'auth-modal-tab-active'
                : 'auth-modal-tab-inactive'
            }`}
            onClick={() => setActiveTab('signup')}
          >
            Sign Up
          </button>
        </div>

        {/* Form Content */}
        <div className="p-6">
          {errors.general && (
            <div className="mb-4 p-3 bg-red-50 dark:bg-red-900/20 text-red-700 dark:text-red-300 rounded-lg animate-fade-in">
              {errors.general}
            </div>
          )}

          <form onSubmit={handleSubmit}>
            {activeTab === 'signup' && (
              <div className="mb-4 animate-fade-in">
                <label htmlFor="name" className="block mb-2 text-sm font-medium auth-modal-label">
                  Full Name
                </label>
                <input
                  type="text"
                  id="name"
                  name="name"
                  value={formData.name}
                  onChange={handleInputChange}
                  className={`w-full px-3 py-2 border rounded-lg focus:outline-none focus:ring-2 transition-colors duration-200 ${
                    errors.name
                      ? 'border-red-500 focus:ring-red-500'
                      : 'auth-modal-input'
                  }`}
                  placeholder="Enter your full name"
                />
                {errors.name && <p className="mt-1 text-sm text-red-600 dark:text-red-400 animate-fade-in">{errors.name}</p>}
              </div>
            )}

            <div className="mb-4 animate-fade-in">
              <label htmlFor="email" className="block mb-2 text-sm font-medium auth-modal-label">
                Email Address
              </label>
              <input
                type="email"
                id="email"
                name="email"
                value={formData.email}
                onChange={handleInputChange}
                className={`w-full px-3 py-2 border rounded-lg focus:outline-none focus:ring-2 transition-colors duration-200 ${
                  errors.email
                    ? 'border-red-500 focus:ring-red-500'
                    : 'auth-modal-input'
                }`}
                placeholder="you@example.com"
              />
              {errors.email && <p className="mt-1 text-sm text-red-600 dark:text-red-400 animate-fade-in">{errors.email}</p>}
            </div>

            <div className="mb-4 animate-fade-in">
              <label htmlFor="password" className="block mb-2 text-sm font-medium auth-modal-label">
                Password
              </label>
              <input
                type="password"
                id="password"
                name="password"
                value={formData.password}
                onChange={handleInputChange}
                className={`w-full px-3 py-2 border rounded-lg focus:outline-none focus:ring-2 transition-colors duration-200 ${
                  errors.password
                    ? 'border-red-500 focus:ring-red-500'
                    : 'auth-modal-input'
                }`}
                placeholder="••••••••"
              />
              {errors.password && <p className="mt-1 text-sm text-red-600 dark:text-red-400 animate-fade-in">{errors.password}</p>}
            </div>

            {activeTab === 'signup' && (
              <>
                <div className="mb-4 animate-fade-in">
                  <label htmlFor="softwareBackground" className="block mb-2 text-sm font-medium auth-modal-label">
                    Software Background
                  </label>
                  <select
                    id="softwareBackground"
                    name="softwareBackground"
                    value={formData.softwareBackground}
                    onChange={handleInputChange}
                    className={`w-full px-3 py-2 border rounded-lg focus:outline-none focus:ring-2 transition-colors duration-200 ${
                      errors.softwareBackground
                        ? 'border-red-500 focus:ring-red-500'
                        : 'auth-modal-input'
                    }`}
                  >
                    <option value="">Select your software background</option>
                    <option value="ros">ROS/ROS2</option>
                    <option value="python">Python</option>
                    <option value="cpp">C++</option>
                    <option value="javascript">JavaScript/Node.js</option>
                    <option value="cuda">CUDA/DL Frameworks</option>
                    <option value="other">Other</option>
                  </select>
                  {errors.softwareBackground && (
                    <p className="mt-1 text-sm text-red-600 dark:text-red-400 animate-fade-in">{errors.softwareBackground}</p>
                  )}
                </div>

                <div className="mb-4 animate-fade-in">
                  <label className="block mb-2 text-sm font-medium auth-modal-label">
                    Hardware Available
                  </label>
                  <div className="space-y-2">
                    {[
                      { value: 'jetson_nano', label: 'Jetson Nano' },
                      { value: 'jetson_orin', label: 'Jetson Orin' },
                      { value: 'raspberry_pi', label: 'Raspberry Pi' },
                      { value: 'nuc', label: 'Intel NUC' },
                      { value: 'desktop_gpu', label: 'Desktop with GPU' },
                      { value: 'cloud_gpu', label: 'Cloud GPU (AWS/Azure/GCP)' },
                      { value: 'none', label: 'None available' }
                    ].map((option) => (
                      <div key={option.value} className="flex items-center animate-fade-in">
                        <input
                          type="checkbox"
                          id={`hw-${option.value}`}
                          checked={formData.hardwareAvailable.includes(option.value)}
                          onChange={() => handleHardwareChange(option.value)}
                          className="h-4 w-4 text-blue-600 focus:ring-blue-500 border-gray-300 rounded transition-colors duration-200"
                        />
                        <label htmlFor={`hw-${option.value}`} className="ml-2 text-sm auth-modal-label transition-colors duration-200">
                          {option.label}
                        </label>
                      </div>
                    ))}
                  </div>
                  {errors.hardwareAvailable && (
                    <p className="mt-1 text-sm text-red-600 dark:text-red-400 animate-fade-in">{errors.hardwareAvailable}</p>
                  )}
                </div>

                <div className="mb-4 animate-fade-in">
                  <label htmlFor="experienceLevel" className="block mb-2 text-sm font-medium auth-modal-label">
                    Experience Level
                  </label>
                  <select
                    id="experienceLevel"
                    name="experienceLevel"
                    value={formData.experienceLevel}
                    onChange={handleInputChange}
                    className={`w-full px-3 py-2 border rounded-lg focus:outline-none focus:ring-2 transition-colors duration-200 ${
                      errors.experienceLevel
                        ? 'border-red-500 focus:ring-red-500'
                        : 'auth-modal-input'
                    }`}
                  >
                    <option value="">Select your experience level</option>
                    <option value="beginner">Beginner</option>
                    <option value="intermediate">Intermediate</option>
                    <option value="advanced">Advanced</option>
                  </select>
                  {errors.experienceLevel && (
                    <p className="mt-1 text-sm text-red-600 dark:text-red-400 animate-fade-in">{errors.experienceLevel}</p>
                  )}
                </div>
              </>
            )}

            <button
              type="submit"
              disabled={isLoading}
              className="w-full py-3 px-4 bg-blue-600 hover:bg-blue-700 text-white font-medium rounded-lg transition-all duration-300 focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-blue-500 dark:focus:ring-offset-gray-800 disabled:opacity-50 transform hover:scale-[1.02] active:scale-[0.98]"
            >
              {isLoading ? (
                <span className="flex items-center justify-center">
                  <svg className="animate-spin -ml-1 mr-3 h-5 w-5 text-white" xmlns="http://www.w3.org/2000/svg" fill="none" viewBox="0 0 24 24">
                    <circle className="opacity-25" cx="12" cy="12" r="10" stroke="currentColor" strokeWidth="4"></circle>
                    <path className="opacity-75" fill="currentColor" d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4zm2 5.291A7.962 7.962 0 014 12H0c0 3.042 1.135 5.824 3 7.938l3-2.647z"></path>
                  </svg>
                  Processing...
                </span>
              ) : activeTab === 'login' ? (
                'Sign In'
              ) : (
                'Sign Up'
              )}
            </button>
          </form>
        </div>
      </div>
    </div>
  );
};

export default AuthModal;
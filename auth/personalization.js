/**
 * Content Personalization Service
 * Adjusts chapter content based on user profile
 */

// Helper function to determine GPU VRAM
function getGpuVram(gpu) {
  if (!gpu) return 0;
  
  const gpuVramMap = {
    // Common GPUs and their VRAM in GB
    'RTX 4090': 24,
    'RTX 4080': 16,
    'RTX 4070 Ti': 12,
    'RTX 4070': 12,
    'RTX 4060 Ti': 8,
    'RTX 4060': 8,
    'RTX 3090': 24,
    'RTX 3080': 10,
    'RTX 3070': 8,
    'RTX 3060 Ti': 8,
    'RTX 3060': 12,
    'RTX 2080 Ti': 11,
    'RTX 2080': 8,
    'RTX 2070': 8,
    'RTX 2060': 6,
    'GTX 1080 Ti': 11,
    'GTX 1080': 8,
    'GTX 1070': 8,
    'GTX 1060': 6,
    'V100': 16,
    'A100': 40,
    'H100': 80,
    'T4': 16,
  };
  
  // Try to match part of the GPU string
  for (const [gpuModel, vram] of Object.entries(gpuVramMap)) {
    if (gpu.toUpperCase().includes(gpuModel.toUpperCase())) {
      return vram;
    }
  }
  
  // Default to 0 if not found
  return 0;
}

/**
 * Personalize chapter content based on user profile
 * @param {string} chapterMarkdown - Original chapter content in markdown
 * @param {Object} userProfile - User profile with preferences
 * @returns {string} - Personalized chapter content
 */
export function personalizeChapterContent(chapterMarkdown, userProfile) {
  let personalizedContent = chapterMarkdown;
  
  if (!userProfile) {
    return chapterMarkdown; // Return original if no profile
  }
  
  // Determine if user has high-end GPU (for VRAM-intensive tasks)
  const gpuVram = getGpuVram(userProfile.gpu);
  const hasHighVramGpu = gpuVram >= 12; // 12GB or more
  
  // Adjust recommended labs based on environment
  if (userProfile.primary_os === 'Web Browser' || userProfile.primary_os === 'Cloud') {
    // Replace local lab instructions with cloud alternatives
    personalizedContent = replaceLabs(personalizedContent, 'cloud');
  } else {
    // Keep local lab instructions
    personalizedContent = replaceLabs(personalizedContent, 'local');
  }
  
  // Adjust hardware checklists based on user's hardware
  personalizedContent = adjustHardwareChecklist(personalizedContent, userProfile.hardware_owned);
  
  // Adjust code snippets based on GPU capability
  if (!hasHighVramGpu) {
    // Replace heavy simulation code with cloud-friendly stubs
    personalizedContent = replaceHeavySimulationCode(personalizedContent);
  }
  
  // Adjust complexity based on experience level
  personalizedContent = adjustComplexity(personalizedContent, userProfile.experience_level);
  
  // Add domain-specific examples
  if (userProfile.main_domain) {
    personalizedContent = addDomainSpecificExamples(personalizedContent, userProfile.main_domain);
  }
  
  return personalizedContent;
}

/**
 * Replace lab instructions based on environment
 */
function replaceLabs(content, environment) {
  if (environment === 'cloud') {
    return content
      .replace(/```bash\n(npm install|pip install|yarn add).*?\n```/gs, 
        '```bash\n# Cloud environment setup\n# Use the in-browser terminal\n# Packages pre-installed in cloud environment\n```')
      .replace(/Run locally/g, 'Run in cloud editor')
      .replace(/Open your local/g, 'Open the cloud editor');
  }
  return content;
}

/**
 * Adjust hardware checklists based on user's hardware
 */
function adjustHardwareChecklist(content, hardwareOwned) {
  if (!hardwareOwned || hardwareOwned.length === 0) {
    return content;
  }
  
  // Add personalized hardware checklist
  const hardwareChecklist = hardwareOwned.map(hw => `- [x] ${hw} (available)`).join('\n');
  
  // Replace generic hardware checklist with personalized one
  content = content.replace(
    /## Hardware Requirements.*?## /s,
    (match) => {
      const intro = match.substring(0, match.indexOf('## '));
      return `${intro}
  
### Your Hardware
${hardwareChecklist}

### Recommended Next Steps
${hardwareOwned.includes('Jetson') ? '* Use Jetson for edge AI applications' : ''}
${hardwareOwned.includes('RealSense') ? '* Integrate RealSense depth sensing' : ''}
${hardwareOwned.includes('Unitree') ? '* Program your Unitree robot' : ''}

## `;
    }
  );
  
  return content;
}

/**
 * Replace heavy simulation code with cloud-friendly stubs
 */
function replaceHeavySimulationCode(content) {
  return content
    .replace(/```python\n.*?simulation\.run\(\).*?\n```/gs, 
      `~~~python
# Cloud-friendly alternative (GPU VRAM < 12GB detected)
# Running heavy simulation on cloud GPU
result = run_cloud_simulation()
print("Simulation completed on cloud GPU")
~~~`)
    .replace(/```python\n.*?large_model =.*?load_model.*?\n.*?\.to\("cuda"\).*?\n```/gs,
      `~~~python
# Cloud-friendly model loading (GPU VRAM < 12GB detected)
# Using optimized model for cloud GPU
model = load_optimized_model()
# Model automatically configured for available VRAM
~~~`);
}

/**
 * Adjust content complexity based on experience level
 */
function adjustComplexity(content, experienceLevel) {
  if (experienceLevel === 'Beginner') {
    // Add more explanations for beginners
    content = content
      .replace(/## (?!Introduction|Overview)/g, '### ') // Demote main headings
      .replace(/### (?!Prerequisites|Requirements)/g, '#### ') // Demote sub-headings
      .replace(/\*\*Advanced:\*\*/g, '**Beginner-Friendly:**');
  } else if (experienceLevel === 'Advanced') {
    // Add more technical depth for advanced users
    content = content
      .replace(/\*\*Beginner-Friendly:\*\*/g, '**Advanced:**')
      .replace(/For more details, see the documentation/g, 
        `For more details, see the documentation.\n\n**Technical Note:** Consider exploring the underlying implementation.`);
  }
  
  return content;
}

/**
 * Add domain-specific examples
 */
function addDomainSpecificExamples(content, domain) {
  if (!domain) return content;
  
  const domainExamples = {
    'Robotics': [
      '* Robot kinematics examples',
      '* Control systems implementation',
      '* Sensor fusion techniques'
    ],
    'Computer Vision': [
      '* Image processing pipelines',
      '* Neural network optimization',
      '* Real-time inference techniques'
    ],
    'Embedded': [
      '* Microcontroller programming',
      '* Low-power system design',
      '* Hardware-software co-design'
    ]
  };
  
  if (domainExamples[domain]) {
    const examplesList = domainExamples[domain].map(ex => `- ${ex}`).join('\n');
    
    // Add domain-specific examples to relevant sections
    if (content.includes('## Examples') || content.includes('## Sample Code')) {
      content = content.replace(
        /(## Examples|## Sample Code).*?\n(?=\n## |\n$)/s,
        (match) => `${match}\n### ${domain}-Specific Examples\n${examplesList}\n`
      );
    } else {
      // Add examples section if none exists
      content += `\n## ${domain}-Specific Examples\n${examplesList}\n`;
    }
  }
  
  return content;
}

// Export for use in other modules
export default personalizeChapterContent;
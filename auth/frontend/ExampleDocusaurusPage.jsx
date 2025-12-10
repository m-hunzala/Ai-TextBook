// ExampleDocusaurusPage.js - Example of how to use personalization in a Docusaurus page

import React from 'react';
import PersonalizedChapter from './PersonalizedChapter';

// Example Docusaurus page component with personalization
const ExampleChapterPage = () => {
  // Example markdown content with special comment tags for personalization
  const chapterMarkdown = `
# Introduction to Robotics Simulation

This chapter introduces you to robotics simulation concepts.

<!-- BEGINNER_HIDE_START -->
## Advanced Mathematical Concepts

For advanced users, we'll dive into the mathematical foundations...
<!-- BEGINNER_HIDE_END -->

<!-- COMPLEX_EXPLANATION_START -->
The simulation process involves complex algorithms that model physical properties with high precision, including collision detection, joint constraints, and dynamics calculations.
<!-- COMPLEX_EXPLANATION_END -->

### Basic Steps

1. Set up your simulation environment
2. Create your robot model
3. Configure physics properties
4. Run the simulation

<!-- ISAAC_SIM_ADVANCED_START -->
## Advanced Isaac Sim Steps

For users with RTX hardware, you can enable advanced rendering features:

- Path tracing for photorealistic results
- Real-time denoising
- Advanced lighting models
<!-- ISAAC_SIM_ADVANCED_END -->

<!-- JETSON_DEPLOYMENT_START -->
## Jetson Deployment

To deploy on Jetson hardware:

- Optimize for ARM architecture
- Configure GPU acceleration
- Set up Jetson-specific libraries
<!-- JETSON_DEPLOYMENT_END -->
  `;

  return (
    <div className="doc-page">
      <header className="doc-header">
        <h1>Robotics Simulation Guide</h1>
      </header>
      
      <main>
        <PersonalizedChapter 
          chapterId="robotics-simulation" 
          originalMarkdown={chapterMarkdown} 
        />
      </main>
    </div>
  );
};

export default ExampleChapterPage;
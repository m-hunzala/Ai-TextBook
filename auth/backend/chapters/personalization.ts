import { userMetadata } from "../schema.ts";

interface UserMetadata {
  softwareBackground?: string;
  hardwareAvailable?: string[];
  experienceLevel?: string;
}

// Main function to apply personalization rules to markdown content
export const applyPersonalizationRules = (markdown: string, userMeta: typeof userMetadata.$inferSelect): string => {
  let personalizedContent = markdown;

  // Apply experience level transformations
  if (userMeta.experienceLevel) {
    personalizedContent = applyExperienceLevelTransformations(personalizedContent, userMeta.experienceLevel);
  }

  // Apply hardware-specific transformations
  if (userMeta.hardwareAvailable && Array.isArray(userMeta.hardwareAvailable)) {
    for (const hardware of userMeta.hardwareAvailable) {
      personalizedContent = applyHardwareTransformations(personalizedContent, hardware);
    }
  }

  // Apply any software background transformations if needed
  if (userMeta.softwareBackground) {
    personalizedContent = applySoftwareBackgroundTransformations(personalizedContent, userMeta.softwareBackground);
  }

  return personalizedContent;
};

// Apply transformations based on experience level
const applyExperienceLevelTransformations = (content: string, experienceLevel: string): string => {
  switch (experienceLevel.toLowerCase()) {
    case 'beginner':
      // For beginners, simplify complex sections and hide advanced content
      content = hideAdvancedSections(content);
      content = simplifyComplexExplanations(content);
      break;
    case 'intermediate':
      // For intermediate users, show moderate detail
      content = moderateDetail(content);
      break;
    case 'advanced':
      // For advanced users, show all content including advanced sections
      content = showAdvancedSections(content);
      break;
    default:
      // Default to intermediate level if not specified
      content = moderateDetail(content);
  }

  return content;
};

// Apply transformations based on hardware available
const applyHardwareTransformations = (content: string, hardware: string): string => {
  switch (hardware.toLowerCase()) {
    case 'rtx_40x':
    case 'rtx_30x':
      // Show advanced Isaac Sim steps for RTX users
      content = showAdvancedIsaacSimSteps(content);
      break;
    case 'jetson_orin':
      // Include Jetson deployment subsection for Jetson users
      content = includeJetsonDeployment(content);
      break;
    case 'none':
      // No special changes for users with no hardware
      break;
    default:
      // Do nothing for other hardware
  }

  return content;
};

// Apply transformations based on software background
const applySoftwareBackgroundTransformations = (content: string, softwareBackground: string): string => {
  // For now, we can use the software background for more specific personalization
  // This could be expanded based on specific keywords in the background
  return content;
};

// Helper functions for transformations

// Hide advanced sections for beginners
const hideAdvancedSections = (content: string): string => {
  // Remove content between <!-- BEGINNER_HIDE_START --> and <!-- BEGINNER_HIDE_END -->
  return content.replace(/<!--\s*BEGINNER_HIDE_START\s*-->([\s\S]*?)<!--\s*BEGINNER_HIDE_END\s*-->/g, '');
};

// Simplify complex explanations for beginners
const simplifyComplexExplanations = (content: string): string => {
  // Replace complex sections with simplified versions
  content = content.replace(/<!--\s*COMPLEX_EXPLANATION_START\s*-->([\s\S]*?)<!--\s*COMPLEX_EXPLANATION_END\s*-->/g, 
    '<!-- SIMPLE_EXPLANATION_START -->\n\n_Simplified explanation for beginners._\n\n<!-- SIMPLE_EXPLANATION_END -->');
  
  // Reduce the number of detailed steps
  content = content.replace(/### Step \d+: (.*?)(?=\n### Step \d+: |$)/gs, 
    (match, stepTitle) => `* ${stepTitle}\n\n`);
  
  return content;
};

// Show moderate detail for intermediate users
const moderateDetail = (content: string): string => {
  // Show moderate level by removing beginner-specific simplifications
  // and leaving standard content as is
  return content;
};

// Show advanced sections for advanced users
const showAdvancedSections = (content: string): string => {
  // Reveal advanced content that might be collapsed for others
  content = content.replace(/<!--\s*ADVANCED_CONTENT_COLLAPSED\s*-->/g, '');
  return content;
};

// Show advanced Isaac Sim steps for RTX users
const showAdvancedIsaacSimSteps = (content: string): string => {
  // Reveal Isaac Sim content for RTX users
  content = content.replace(/<!--\s*ISAAC_SIM_ADVANCED_START\s*-->([\s\S]*?)<!--\s*ISAAC_SIM_ADVANCED_END\s*-->/g, 
    '\n## Advanced Isaac Sim Steps\n\n$1\n');
  return content;
};

// Include Jetson deployment subsection for Jetson users
const includeJetsonDeployment = (content: string): string => {
  // Reveal Jetson deployment content
  content = content.replace(/<!--\s*JETSON_DEPLOYMENT_START\s*-->([\s\S]*?)<!--\s*JETSON_DEPLOYMENT_END\s*-->/g, 
    '\n## Jetson Deployment\n\n$1\n');
  return content;
};
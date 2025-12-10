/**
 * Example: Preserving Fenced Code Blocks During Translation
 * 
 * This example demonstrates how the translation system preserves
 * fenced code blocks while translating surrounding text.
 */

// Original English markdown content
const originalMarkdown = `
# Introduction to Robotics

This chapter introduces you to robotics concepts.

## Code Example

Here's a Python function to control a robot:

\`\`\`python
def move_robot(direction, speed=1.0):
    """Move the robot in a given direction at a specified speed."""
    if direction == "forward":
        print(f"Moving forward at speed {speed}")
    elif direction == "backward":
        print(f"Moving backward at speed {speed}")
    else:
        print("Invalid direction")
\`\`\`

## Advanced Concepts

For advanced users, we'll dive into complex algorithms that model physical properties with high precision.

\`\`\`javascript
class RobotController {
  constructor(name) {
    this.name = name;
    this.position = { x: 0, y: 0 };
  }
  
  move(x, y) {
    this.position.x += x;
    this.position.y += y;
    return this.position;
  }
}
\`\`\`

More text content that should be translated.
`;

// Expected translated Urdu markdown content
const translatedMarkdown = `
# روبوٹکس کا تعارف

یہ باب آپ کو روبوٹکس کے تصورات سے متعارف کرواتا ہے۔

## کوڈ کی مثال

یہ ایک پائی تھون فنکشن ہے جو روبوٹ کو کنٹرول کرتا ہے:

\`\`\`python
def move_robot(direction, speed=1.0):
    """Move the robot in a given direction at a specified speed."""
    if direction == "forward":
        print(f"Moving forward at speed {speed}")
    elif direction == "backward":
        print(f"Moving backward at speed {speed}")
    else:
        print("Invalid direction")
\`\`\`

## اعلیٰ تصورات

اعلیٰ صارفین کے لیے، ہم پیچیدہ الخوارزمیوں میں غور کریں گے جو جسمانی خصوصیات کو زیادہ درستی کے ساتھ ماڈل کرتے ہیں۔

\`\`\`javascript
class RobotController {
  constructor(name) {
    this.name = name;
    this.position = { x: 0, y: 0 };
  }
  
  move(x, y) {
    this.position.x += x;
    this.position.y += y;
    return this.position;
  }
}
\`\`\`

زیادہ متن کا مواد جس کا ترجمہ کیا جانا چاہیے۔
`;

/**
 * The translation system works as follows:
 * 
 * 1. Extract all fenced code blocks (```...```) before translation
 * 2. Translate only the non-code content
 * 3. Restore the original code blocks in their exact positions
 * 
 * This ensures that:
 * - Code syntax remains correct
 * - Language-specific keywords are preserved
 * - Indentation and formatting are maintained
 * - The code will still execute properly after translation
 */

// Example implementation showing the extraction process
function demonstrateCodeBlockPreservation() {
  const original = `# Hello World

This is a description that should be translated.

\`\`\`python
print("Hello, world!")
def example():
    return "This code should NOT be translated"
\`\`\`

More content here.
`;

  console.log("Original content:");
  console.log(original);
  console.log("\n" + "=".repeat(50) + "\n");

  // Extract code blocks (simulated)
  const extracted = extractCodeBlocks(original);
  console.log("Content without code blocks (ready for translation):");
  console.log(extracted.contentWithoutCode);
  console.log("\nExtracted code blocks:");
  extracted.codeBlocks.forEach((block, i) => {
    console.log(`Block ${i + 1}:`, block);
  });
  console.log("\n" + "=".repeat(50) + "\n");

  // After translation (simulated)
  const translatedText = ".ur. ہیل ورلڈ\\n\\nیہ ایک تفصیل ہے جس کا ترجمہ کیا جانا چاہیے.\\n\\n__CODE_BLOCK_PLACEHOLDER_0__\\n\\nیہاں مزید مواد.";
  console.log("Translated text (with placeholders):");
  console.log(translatedText);
  console.log("\n" + "=".repeat(50) + "\n");

  // Restore code blocks
  const final = restoreCodeBlocks(translatedText, extracted.codeBlocks);
  console.log("Final content with restored code blocks:");
  console.log(final);
}

// Helper function to extract code blocks
function extractCodeBlocks(markdown) {
  const codeBlockRegex = /```[\s\S]*?```/g;
  const codeBlocks = markdown.match(codeBlockRegex) || [];
  const contentWithoutCode = markdown.replace(codeBlockRegex, (match, offset) => {
    const placeholder = `__CODE_BLOCK_PLACEHOLDER_${offset}__`;
    return placeholder;
  });

  return {
    contentWithoutCode,
    codeBlocks
  };
}

// Helper function to restore code blocks
function restoreCodeBlocks(translatedContent, originalCodeBlocks) {
  let result = translatedContent;
  originalCodeBlocks.forEach((block, index) => {
    const placeholder = `__CODE_BLOCK_PLACEHOLDER_${index}__`;
    result = result.replace(placeholder, block);
  });
  return result;
}

// Run the demonstration
demonstrateCodeBlockPreservation();

export { extractCodeBlocks, restoreCodeBlocks, originalMarkdown, translatedMarkdown };
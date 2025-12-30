# Data Model: Documentation Code Blocks

**Feature**: 1-fix-mdx-compilation
**Created**: 2025-12-30
**Purpose**: Define structure for properly formatted code blocks in MDX documentation

## Entities

### CodeBlock
**Description**: A properly formatted code example in MDX documentation

**Fields**:
- `id`: string (unique identifier for the code block)
- `language`: string (programming language identifier, e.g., "python", "javascript", "bash")
- `content`: string (the actual code content)
- `isValidMdx`: boolean (whether it passes MDX parsing without errors)
- `syntaxHighlighting`: boolean (whether it renders with proper syntax highlighting)
- `isSafePattern`: boolean (whether it uses safe patterns that don't confuse MDX parser)

**Relationships**:
- Belongs to: MDXFile
- Contains: CodeLines

### MDXFile
**Description**: An MDX documentation file containing code blocks

**Fields**:
- `path`: string (file path location)
- `title`: string (documentation page title)
- `codeBlocks`: array of CodeBlock (list of code blocks in the file)
- `buildsSuccessfully`: boolean (whether file builds without MDX errors)
- `lastModified`: datetime (timestamp of last change)

**Relationships**:
- Contains: CodeBlock
- Belongs to: DocumentationSection

### DocumentationSection
**Description**: A section of the documentation (e.g., tutorials, guides)

**Fields**:
- `name`: string (section name)
- `files`: array of MDXFile (list of MDX files in the section)
- `buildStatus`: string (overall build status: "success", "error", "warning")

**Relationships**:
- Contains: MDXFile

## Validation Rules

### From Requirements
- **FR-003**: CodeBlock.language must be a valid language identifier for syntax highlighting
- **FR-002**: CodeBlock.content must use safe patterns that don't confuse MDX parser
- **FR-005**: CodeBlock.syntaxHighlighting must be true for proper rendering

### Pattern Validation
- **SafeFStringPattern**: Code blocks should avoid complex f-strings with nested braces
- **FormatMethodPattern**: Prefer `.format()` method over complex f-strings
- **ProperTagging**: All code blocks must have proper language tags (```python, etc.)

## State Transitions

### CodeBlock States
1. **RawContent** → **Validated** (when MDX parsing is verified)
   - Trigger: Build process completion
   - Validation: No MDX compilation errors

2. **Validated** → **Rendered** (when syntax highlighting is confirmed)
   - Trigger: Visual inspection
   - Validation: Proper syntax highlighting displayed

## Constraints

### Safety Constraints
- CodeBlock.content must not contain patterns that trigger MDX parser errors
- CodeBlock.language must be supported by Docusaurus syntax highlighter
- All f-strings with complex nested braces must be converted to `.format()` method

### Quality Constraints
- All code examples must maintain educational value
- Code formatting must remain readable and clear
- Documentation accuracy must be preserved during transformation
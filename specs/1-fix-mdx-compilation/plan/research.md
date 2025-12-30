# Research: Resolving MDX Compilation Error

**Feature**: 1-fix-mdx-compilation
**Created**: 2025-12-30
**Status**: Complete

## Decision: Problem Identification and Solution Approach

**Rationale**: The MDX compilation errors were caused by complex Python f-strings with nested braces that confused the MDX parser. The parser expected only import/export statements but encountered what it interpreted as JavaScript function declarations. The solution was to replace complex f-strings with the `.format()` method which avoids the problematic nested brace patterns.

**Alternatives Considered**:
1. **Leave f-strings as-is**: Would keep the compilation errors, making the build unusable
2. **Remove Python code blocks entirely**: Would eliminate important documentation content
3. **Use different code block syntax**: Would not address the core parsing issue
4. **Replace f-strings with `.format()` method**: Successfully resolves parsing issues while preserving content (chosen)

## Problem Analysis

### Root Cause
The MDX parser in Docusaurus was encountering Python f-strings with complex nested braces like:
```python
prompt = f"""
{{
  "key": "{variable}"
}}
"""
```

The parser misinterpreted the nested braces as JavaScript template literals or function declarations instead of properly formatted Python code within a code block.

### Affected Files
Through investigation, the primary problematic file was identified:
- `docs/capstone/end-to-end.md` - Contains complex f-strings in the `generate_llm_plan` method

### Solution Applied
The problematic f-string was replaced with a template string using the `.format()` method:
- **Before**: Complex f-string with nested braces
- **After**: Template string with `.format()` method

## MDX Parser Behavior

### Issues Identified
1. **Nested Braces**: The MDX parser gets confused by `{{ }}` patterns in f-strings
2. **Multi-line F-strings**: Complex multiline f-strings with nested structures cause parsing issues
3. **Template Literal Confusion**: Parser may confuse Python f-strings with JavaScript template literals

### Safe Patterns
1. **Template Strings with `.format()`**: Safe alternative to complex f-strings
2. **Simple f-strings**: Basic f-strings without complex nesting work fine
3. **Properly Closed Code Blocks**: Ensuring code blocks have proper language tags and closing

## Best Practices for Python in MDX

### Recommended Approaches
1. **Use `.format()` for complex strings**: Avoids nested brace confusion
2. **Proper Language Tags**: Always use ```python for Python code blocks
3. **Simple f-strings**: Use basic f-strings only for simple variable insertion
4. **Template Strings**: For complex formatting, use triple-quoted templates with `.format()`

### Patterns to Avoid
1. **Complex Nested Braces**: `f"{{key: {value}}}"` patterns
2. **Multiline f-strings with JSON**: Complex multiline f-strings that look like JS objects
3. **Unbalanced Braces**: Any brace patterns that might confuse the parser

## Verification Process

### Testing Approach
1. **Build Verification**: Run `npm run build` to ensure no MDX errors
2. **Development Server**: Test with `npm start` to ensure proper rendering
3. **Visual Inspection**: Verify code blocks render with proper syntax highlighting
4. **Content Validation**: Ensure documentation content remains accurate

### Success Indicators
- Build process completes without MDX compilation errors
- Code blocks render with proper syntax highlighting
- Development server starts and serves pages correctly
- Documentation content remains accurate and useful
from llava.model.builder import load_pretrained_model
from transformers import AutoProcessor
from PIL import Image
import torch

model_name = "lmms-lab/llava-onevision-qwen2-7b-ov"
model_base = "qwen2-7b"
model_path = 

model, image_processor = load_pretrained_model(
    pretrained=model_name, 
    model_base=model_base, 
    model_name=model_name, 
    device_map="auto"
)

model.to("cuda" if torch.cuda.is_available() else "cpu")
model.eval()

# model = resnet50(weights=None)
# model.load_state_dict(torch.load("./resnet50-0676ba61.pth"))

image_path = "/home/hs/disk1/evaluation/ours/images/banana.jpg"
image = Image.open(image_path).convert("RGB")

prompt = "Describe the content of the image"
inputs = processor(images=image, text=prompt, return_tensors="pt").to(device)

with torch.no_grad():
    outputs = model.generate(
        input_ids=inputs["input_ids"],
        pixel_values=inputs["pixel_values"],
        max_length=50,
        num_beams=5,
    )

generated_text = tokenizer.decode(outputs[0], skip_special_tokens=True)
print("Generated Description:", generated_text)